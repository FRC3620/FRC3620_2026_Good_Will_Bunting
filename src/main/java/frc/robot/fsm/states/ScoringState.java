package frc.robot.fsm.states;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Helpers.FieldTriggers;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.fsm.StateTransition;
import frc.robot.fsm.SuperState;

public class ScoringState extends SuperState {

    @Override
    public void onEnter() {
        // Code to run when entering the Scoring state
        ParallelCommandGroup scoringCommand = RobotContainer.intakeRollerSubsystem.rollersOn()

                .alongWith(
                        RobotContainer.conveyerSubsystem.setDutyCycle(0.9))

                .alongWith(
                        RobotContainer.turretSubsystem.createSetAngleToTargetCommand(
                                AllianceFlipUtil.apply(
                                        ShotCalculator.FieldTargets.BLUE_HUB.getTargetPosition().toTranslation2d()),
                                () -> RobotContainer.swerveSubsystem.getState().Pose,
                                () -> frc.robot.Helpers.ShotCalculator.calculateRobotVelocity(
                                        RobotContainer.swerveSubsystem.getKinematics(),
                                        RobotContainer.swerveSubsystem.getState(),
                                        RobotContainer.swerveSubsystem.getPigeon2().getRotation2d())))

                .alongWith(
                        RobotContainer.shooterSubsystem.createSetSpeedToTargetCommand(
                                ShotCalculator.FieldTargets.BLUE_HUB.getTargetPosition(),
                                () -> AllianceFlipUtil.apply(
                                        RobotContainer.swerveSubsystem.getState().Pose),
                                () -> frc.robot.Helpers.AllianceFlipUtil.apply(
                                        frc.robot.Helpers.ShotCalculator.calculateRobotVelocity(
                                                RobotContainer.swerveSubsystem.getKinematics(),
                                                RobotContainer.swerveSubsystem.getState(),
                                                RobotContainer.swerveSubsystem.getPigeon2().getRotation2d()))))

                .alongWith(
                        RobotContainer.shooterHoodSubsystem.createAutoAngleToTargetCommand(
                                new Translation3d(
                                        Feet.of(15.17),
                                        Feet.of(13.235),
                                        Feet.of(6)),
                                () -> AllianceFlipUtil.apply(
                                        RobotContainer.swerveSubsystem.getState().Pose),
                                () -> AllianceFlipUtil.apply(
                                        ShotCalculator.calculateRobotVelocity(
                                                RobotContainer.swerveSubsystem.getKinematics(),
                                                RobotContainer.swerveSubsystem.getState(),
                                                RobotContainer.swerveSubsystem.getPigeon2().getRotation2d()))))

                .alongWith(
                        RobotContainer.preshooterSubsystem
                                .createSetVelocityCommand(() -> RPM.of(2000)));
        CommandScheduler.getInstance().schedule(scoringCommand);
    }

    @Override
    public void execute() {
        // Code to run while in the Scoring state

    }

    @Override
    public void onExit() {
        // Code to run when exiting the Scoring state

    }

}