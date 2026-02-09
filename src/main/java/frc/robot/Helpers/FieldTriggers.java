package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.usfirst.frc3620.XBoxConstants;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveSubsystem;

public class FieldTriggers {

        // field halves
        public final Trigger enterOpponentAlliance;
        public final Trigger enterOurAlliance;

        public final Trigger enterOurAllianceZone;
        public final Trigger enterNeutralDepot;
        public final Trigger enterNeutralOutpost;
        public final Trigger enterOpponentDepot;
        public final Trigger enterOpponentOutpost;
        public final Trigger enterDeadZone;
        public final Trigger enterClimbZone;

        public FieldTriggers(Supplier<Pose2d> pSupplier) {
                // if pose supplier returns null use a default pose
                Supplier<Pose2d> safePose = () -> {
                        Pose2d p = pSupplier.get();
                        return p != null ? p : new Pose2d();
                };

                // each half of the field
                enterOpponentAlliance = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) >= 8);
                enterOurAlliance = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 8);

                // field areas
                enterOurAllianceZone = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 4.625594);

                enterNeutralDepot = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 4.625594
                                                && AllianceFlipUtil.applyX(
                                                                safePose.get().getMeasureX().in(Meters)) < 11.887454
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) > 4.763135);
                enterNeutralOutpost = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 4.625594
                                                && AllianceFlipUtil.applyX(
                                                                safePose.get().getMeasureX().in(Meters)) < 11.887454
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) < 3.266821);
                enterOpponentDepot = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 11.887454
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) > 4.763135);
                enterOpponentOutpost = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 11.887454
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) < 3.266821);
                enterDeadZone = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 4.625594
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) > 3.266821
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY()
                                                                                .in(Meters)) < 4.763135);
                enterClimbZone = new Trigger(
                                () -> AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 2.5
                                                && AllianceFlipUtil
                                                                .applyY(safePose.get().getMeasureY().in(Meters)) > 7.1
                                                && AllianceFlipUtil
                                                                .applyX(safePose.get().getMeasureY().in(Meters)) < 9.3);

        }

}
