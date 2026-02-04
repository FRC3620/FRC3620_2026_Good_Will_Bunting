package frc.robot.fsm.states;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fsm.StateTransition;

public class DeadeyeState implements IState {

    public final static TaggedLogger logger = LoggingMaster.getLogger(DeadeyeState.class);

    private List<StateTransition> stateTransitions = new ArrayList<>();

    public DeadeyeState() {
    }

    public void addTransition(StateTransition transition) {
        if (stateTransitions != null) {
            stateTransitions.add(transition);
        }
    }

    @Override
    public void onEnter() {
        // Code to run when entering the Scoring state
        logger.info("Entering Deadeye State");
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
    public Optional<IState> nextState() {
        // Logic to determine the next state
        for (StateTransition transition : stateTransitions) {
            if (transition.triggerToState().getAsBoolean()) {
                logger.info("Transitioning from Deadeye to {}", transition.targetState().getClass().getSimpleName());
                return Optional.of(transition.targetState());
            }
        }
        return Optional.empty();
    }
}