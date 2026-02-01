package frc.robot.fsm;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fsm.states.IState;

public record StateTransition(Trigger triggerToState, IState targetState) {

}
