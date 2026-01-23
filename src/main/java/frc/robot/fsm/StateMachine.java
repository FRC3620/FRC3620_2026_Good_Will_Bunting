package frc.robot.fsm;

import java.util.Optional;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.fsm.states.IState;

public class StateMachine {

    public final static TaggedLogger logger = LoggingMaster.getLogger(StateMachine.class);

    private IState currentState;

    public StateMachine(IState initialState) {
        this.currentState = initialState;
        this.currentState.onEnter();
    }

    public void update() {
        currentState.execute();
        SmartDashboard.putString("FSM Current State", currentState.getClass().getSimpleName());
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