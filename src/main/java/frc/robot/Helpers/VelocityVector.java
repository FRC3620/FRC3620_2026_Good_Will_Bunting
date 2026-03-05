package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class VelocityVector {
        public LinearVelocity xLinearVelocity;
        public LinearVelocity yLinearVelocity;

        public VelocityVector(LinearVelocity xLinearVelocity, LinearVelocity yLinearVelocity) {
            this.xLinearVelocity = xLinearVelocity;
            this.yLinearVelocity = yLinearVelocity;
        }

        public VelocityVector(Distance xDistance, Distance yDistance) {
            this.xLinearVelocity = FeetPerSecond.of(xDistance.in(Feet));
            this.yLinearVelocity = FeetPerSecond.of(yDistance.in(Feet));
        }

        public LinearVelocity getX() {
            return xLinearVelocity;
        }

        public LinearVelocity getY() {
            return yLinearVelocity;
        }

        public LinearVelocity getNorm() {
            return FeetPerSecond.of(Math.hypot(xLinearVelocity.in(FeetPerSecond), yLinearVelocity.in(FeetPerSecond)));
        }

        public Translation2d getTranslation() {
            return new Translation2d(Feet.of(xLinearVelocity.in(FeetPerSecond)), Feet.of(yLinearVelocity.in(FeetPerSecond)));
        }
    }