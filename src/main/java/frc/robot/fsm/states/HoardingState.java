package frc.robot.fsm.states;

import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ConveyerSubsystem;
import frc.robot.fsm.StateTransition;
import frc.robot.fsm.SuperState;

public class HoardingState extends SuperState {



    @Override
    public void onEnter() {
        // Code to run when entering the Scoring state

        Command conveyerOff = RobotContainer.conveyerSubsystem.setDutyCycle(0.0);
        Command agitatorOff = RobotContainer.intakeAgitatorSubsystem.agitatorOff();

        CommandScheduler.getInstance().schedule(conveyerOff.alongWith(agitatorOff));
    
    }

    @Override
    public void execute() {
        // Code to run while in the Scoring state

    }

    @Override
    public void onExit() {
        // Code to run when exiting the Scoring state

    }


    @Override
    public LEDPattern getLEDPattern() {
        return LEDPattern.solid(Color.kRed);
    }
}