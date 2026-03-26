package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
import frc.robot.Subsystems.IntakeShoulderSubsystem;

public class AutoAimShooterCommand extends ParallelCommandGroup {

    public AutoAimShooterCommand(Translation3d target) {

        Supplier<VelocityVector> robotVelocity;

        Supplier<Pose2d> robotPose = () -> AllianceFlipUtil.apply(RobotContainer.swerveSubsystem.getState().Pose);

        if (RobotContainer.questNavSubsystem.getQuestNavConnected()
                && RobotContainer.questNavSubsystem.getQuestNavIsTracking()) {

            robotVelocity = () -> AllianceFlipUtil.apply(ShotCalculator.calculateQuestVelocity(
                    RobotContainer.questNavSubsystem.getQuestNavVX(), RobotContainer.questNavSubsystem.getQuestNavVY(),
                    RobotContainer.questNavSubsystem.getNavQuestPose3d().getRotation().toRotation2d()));

        } else {
            robotVelocity = () -> AllianceFlipUtil.apply(ShotCalculator.calculateRobotVelocity(
                    RobotContainer.swerveSubsystem.getKinematics(),
                    RobotContainer.swerveSubsystem.getState(),
                    RobotContainer.swerveSubsystem.getPigeon2().getRotation2d()));

        }

        Supplier<AngularVelocity> shooterExitSpeed = 
            () -> RobotContainer.shooterSubsystem.getVelocity();
        addCommands(

            RobotContainer.turretSubsystem.createSetAngleToTargetCommand(
                target.toTranslation2d(),
                robotPose,
                robotVelocity
            ),

            RobotContainer.shooterSubsystem.createSetSpeedToTargetCommand(
                target,
                robotPose,
                robotVelocity
            ),

            RobotContainer.shooterHoodSubsystem.createAutoAngleToTargetCommand(
                target,
                robotPose,
                robotVelocity,
                shooterExitSpeed
            ),

            RobotContainer.preshooterSubsystem.createSetVelocityCommand(
                () -> RPM.of(700)
            )
        );
    }
}
