package frc.robot.fsm.states;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PassingState implements IState {
    private Trigger triggerToState;
    private IState targetState;
    private Trigger triggerToState2;
    private IState targetState2;
    private Trigger triggerToState3;
    private IState targetState3;

    public PassingState() {

    }

    public PassingState(Trigger triggerToState, IState targetState) {
        this.triggerToState = triggerToState;
        this.targetState = targetState;
    }

    public PassingState(Trigger triggerToState, IState targetState, Trigger triggerToState2, IState targetState2) {
        this.triggerToState = triggerToState;
        this.targetState = targetState;
        this.triggerToState2 = triggerToState2;
        this.targetState2 = targetState2;
    }

    public PassingState(Trigger triggerToState, IState targetState, Trigger triggerToState2, IState targetState2, Trigger triggerToState3, IState targetState3) {
        this.triggerToState = triggerToState;
        this.targetState = targetState;
        this.triggerToState2 = triggerToState2;
        this.targetState2 = targetState2;
        this.triggerToState3 = triggerToState3;
        this.targetState3 = targetState3;
    }
    
    @Override
    public void onEnter() {
        // Code to run when entering the Passing state
        System.out.println("Entering Passing State");
    }

    @Override
    public void execute() {
        // Code to run while in the Passing state
        System.out.println("Locked onto passing target");
    }

    @Override
    public void onExit() {
        // Code to run when exiting the Passing state
        System.out.println("Exiting Passing State");
    }

    @Override
    public Optional<IState> nextState() {
        // Logic to determine the next state
        if (triggerToState != null && triggerToState.getAsBoolean()) {

            return Optional.of(targetState);

        } else if (triggerToState2 != null && triggerToState2.getAsBoolean()) {

            return Optional.of(targetState2);

        } else if(triggerToState3 != null && triggerToState3.getAsBoolean()) {

            return Optional.of(targetState3);

        } else {

            return Optional.empty();

        }
    }
}