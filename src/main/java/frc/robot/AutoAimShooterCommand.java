package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
import frc.robot.Subsystems.IntakeShoulderSubsystem;

public class AutoAimShooterCommand extends ParallelCommandGroup {

    public AutoAimShooterCommand(Translation3d target) {

        Supplier<Pose2d> robotPose =
            () -> RobotContainer.swerveSubsystem.getState().Pose;

        Supplier<VelocityVector> robotVelocity =
            () -> ShotCalculator.calculateRobotVelocity(
                RobotContainer.swerveSubsystem.getKinematics(),
                RobotContainer.swerveSubsystem.getState(),
                RobotContainer.swerveSubsystem.getPigeon2().getRotation2d()
            );

        addCommands(

            RobotContainer.intakeRollerSubsystem.rollersOn(),

            RobotContainer.intakeShoulderSubsystem.createSetPositionCommand(
                () -> IntakeShoulderSubsystem.IntakeShoulderPositions.OUT.getAngle()
            ),

            RobotContainer.turretSubsystem.createSetAngleToTargetCommand(
                AllianceFlipUtil.apply(target.toTranslation2d()),
                robotPose,
                robotVelocity
            ),

            RobotContainer.shooterSubsystem.createSetSpeedToTargetCommand(
                target,
                () -> AllianceFlipUtil.apply(robotPose.get()),
                () -> AllianceFlipUtil.apply(robotVelocity.get())
            ),

            RobotContainer.shooterHoodSubsystem.createAutoAngleToTargetCommand(
                target,
                () -> AllianceFlipUtil.apply(robotPose.get()),
                () -> AllianceFlipUtil.apply(robotVelocity.get())
            ),

            RobotContainer.preshooterSubsystem.createSetVelocityCommand(
                () -> Units.RPM.of(2000)
            )
        );
    }
}

