package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FieldTriggers {

        // field halves
        public final Trigger enterOpponentAlliance;
        public final Trigger enterOurAlliance;
        public final Trigger enterDepotHalf;
        public final Trigger enterOutpostHalf;

        public final Trigger enterOurAllianceZone;
        public final Trigger enterDepotPass;
        public final Trigger enterOutpostPass;
      //  public final Trigger enterOpponentDepot;
       // public final Trigger enterOpponentOutpost;
        public final Trigger enterDeadZone;
     //   public final Trigger enterClimbZone;

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

                enterDepotHalf = new Trigger(
                        ()-> AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 4.034536
                );
                enterOutpostHalf = new Trigger(
                        ()-> AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) <= 4.034536
                );

                // field areas
                enterOurAllianceZone = new Trigger(
                                () -> (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 4 
                                && AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 1.1)

                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 4
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 4.064)

                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 4
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 2.891536));

                enterDeadZone = new Trigger(
                                () -> (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 1.1
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 2.891536
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 4.064)

                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 4.0132
                                && AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 5.842
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 3.292729
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 4.776343)

                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 5.842
                                && AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 7.03496582
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 3.292729+0.3709035
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 4.776343-0.3709035)
                                
                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 7.03496582
                                && AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) < 8.270494
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 3.292729+0.3709035+0.3
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 4.776343-0.3709035-0.3)
                                
                                || (AllianceFlipUtil.applyX(safePose.get().getMeasureX().in(Meters)) > 11.435334
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) < 4.763135
                                && AllianceFlipUtil.applyY(safePose.get().getMeasureY().in(Meters)) > 3.266821));
        

                enterDepotPass = enterOurAllianceZone.negate().and(enterDeadZone.negate()).and(enterDepotHalf);
                enterOutpostPass = enterOurAllianceZone.negate().and(enterDeadZone.negate()).and(enterOutpostHalf).and(enterDepotPass.negate());

        }

}
