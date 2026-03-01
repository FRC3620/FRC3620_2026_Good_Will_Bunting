package frc.robot.Helpers;

import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class VelocityVector {
        public LinearVelocity xLinearVelocity;
        public LinearVelocity yLinearVelocity;

        public VelocityVector(LinearVelocity xLinearVelocity, LinearVelocity yLinearVelocity) {
            this.xLinearVelocity = xLinearVelocity;
            this.yLinearVelocity = yLinearVelocity;
        }

        LinearVelocity getX() {
            return xLinearVelocity;
        }

        LinearVelocity getY() {
            return yLinearVelocity;
        }

        LinearVelocity getNorm() {
            return FeetPerSecond.of(Math.hypot(xLinearVelocity.in(FeetPerSecond), yLinearVelocity.in(FeetPerSecond)));
        }
    }