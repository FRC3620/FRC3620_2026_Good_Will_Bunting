package frc.robot.fsm.states;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoAimShooterCommand;
import frc.robot.RobotContainer;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.fsm.StateTransition;
import frc.robot.fsm.SuperState;

public class OutpostDeepState extends SuperState {

    /*
     * Command conveyerOn = RobotContainer.conveyerSubsystem
     * .setDutyCycleGated(.8, () -> RobotContainer.shooterSubsystem.atRPM(),
     * () -> RobotContainer.turretSubsystem.atTarget(),
     * () -> RobotContainer.shooterHoodSubsystem.atTarget())
     * .until(() -> !RobotContainer.turretSubsystem.atTarget()).until(
     * () -> !RobotContainer.shooterHoodSubsystem.atTarget());
     * Command agitatorOn = RobotContainer.intakeAgitatorSubsystem.agitatorOn();
     * 
     * Command agitatorBack =
     * RobotContainer.intakeAgitatorSubsystem.agitatorBackwards();
     */

    Command doEverythingCommand = new AutoAimShooterCommand(ShotCalculator.FieldTargets.OP_DEEP_PASS.getTargetPositionSupplier());

    @Override
    public void onEnter() {
        // Code to run when entering the Passing state

        CommandScheduler.getInstance().schedule(doEverythingCommand);

        RobotContainer.setActiveTarget(ShotCalculator.FieldTargets.OP_DEEP_PASS);

    }

    @Override
    public void execute() {
        // Code to run while in the Passing state
    }

    @Override
    public void onExit() {
        // Code to run when exiting the Passing state

    }

    @Override
    public LEDPattern getLEDPattern() {
        return LEDPattern.solid(Color.kPink);
    }

}