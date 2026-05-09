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

public class ParadeShootCommand extends ParallelCommandGroup {

    public ParadeShootCommand() {

        Supplier<AngularVelocity> shooterExitSpeed = () -> RobotContainer.shooterSubsystem.getVelocity();

        addCommands(

                RobotContainer.shooterSubsystem.paradeAutoAim(),

                RobotContainer.shooterHoodSubsystem.paradeAutoAim(),
                        
                RobotContainer.preshooterSubsystem.createSetVelocityCommand(
                        () -> RPM.of(700)));
    }
}
