package frc.robot.fsm.states;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
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

    // --- LED pattern fields ---
    private static final LEDPattern SLOW_BLINK = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5));
    private static final LEDPattern FAST_BLINK = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1));
    private static final LEDPattern DEFAULT_PATTERN = LEDPattern.solid(Color.kRed);

    private double warningStartTime = -1; // -1 means condition not yet triggered
    private boolean wasWarningActive = false;

    @Override
    public void onEnter() {
        // Reset warning state every time we enter hoarding
        warningStartTime = -1;
        wasWarningActive = false;

        Command conveyerOff = RobotContainer.conveyerSubsystem.setDutyCycle(0.0);
        Command agitatorOff = RobotContainer.intakeAgitatorSubsystem.agitatorOff();
        CommandScheduler.getInstance().schedule(conveyerOff.alongWith(agitatorOff));
    }

    @Override
    public void execute() {
        boolean warningActive = isAllianceHubWarningActive(); // your condition here

        // Latch the start time the first moment the condition becomes true
        if (warningActive && !wasWarningActive) {
            warningStartTime = Timer.getFPGATimestamp();
        }

        wasWarningActive = warningActive;
    }

    @Override
    public void onExit() {
        warningStartTime = -1;
        wasWarningActive = false;
    }

    @Override
    public LEDPattern getLEDPattern() {
        if (warningStartTime < 0) {
            // Condition hasn't triggered yet
            return DEFAULT_PATTERN;
        }

        double elapsed = Timer.getFPGATimestamp() - warningStartTime;

        if (elapsed < 5.0) {
            return SLOW_BLINK; // First 5 seconds: slow blink
        } else if (elapsed < 10.0) {
            return FAST_BLINK; // Next 5 seconds: fast blink
        } else {
            // Warning window has passed — return to default or hold fast blink
            return FAST_BLINK;
        }
    }

    /**
     * Replace this with however you detect the hub is 10 seconds from activating.
     * Could be a Timer, a game data field, a subsystem flag, etc.
     */
    private boolean isAllianceHubWarningActive() {
        // e.g. return RobotContainer.fieldSubsystem.getSecondsUntilHubActive() <= 10.0;
        return false;
    }
}