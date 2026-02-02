package frc.robot.fsm.states;

import java.util.Optional;

public interface IState {
    void onEnter();
    void execute();
    void onExit();

    Optional<IState> nextState();

}