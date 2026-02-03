package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Meters;

import org.usfirst.frc3620.XBoxConstants;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveSubsystem;

public class FieldTriggers {

    public final Trigger enterOpponentAlliance;
    public final Trigger enterOurAlliance;

    public FieldTriggers(SwerveSubsystem swerveSubsystem) {

        enterOpponentAlliance = new Trigger(() -> AllianceFlipUtil.applyX(swerveSubsystem.getState().Pose.getMeasureX().in(Meters)) >= 8);

        enterOurAlliance = new Trigger(() -> AllianceFlipUtil.applyX(swerveSubsystem.getState().Pose.getMeasureX().in(Meters)) < 8);



    
    }

}
