package frc.robot.fsm;

import java.util.Optional;

import frc.robot.fsm.states.IState;

public class StateMachine {
    private IState currentState;

    public StateMachine(IState initialState) {
        this.currentState = initialState;
        this.currentState.onEnter();
    }

    public void update() {
        currentState.execute();
        Optional<IState> nextState = currentState.nextState();
        if (nextState.isPresent()) {
            currentState.onExit();
            currentState = nextState.get();
            currentState.onEnter();
        }
    }

    public IState getCurrentState() {
        return currentState;
    }
}