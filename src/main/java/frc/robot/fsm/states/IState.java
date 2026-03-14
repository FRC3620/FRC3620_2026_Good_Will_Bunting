package frc.robot.fsm.states;

import java.util.Optional;

import edu.wpi.first.wpilibj.LEDPattern;

public interface IState {
    void onEnter();
    void execute();
    void onExit();

    Optional<IState> nextState();

    LEDPattern getLEDPattern();
}