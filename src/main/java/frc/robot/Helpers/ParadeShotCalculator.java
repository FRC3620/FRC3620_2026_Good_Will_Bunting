package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simplified shot calculator for parade use.
 * Turret aims using tx from tag 22 (no pose required).
 * Shooter speed and hood angle are derived from distToRobot from tag 22.
 * Robot velocity compensation is omitted (robot is stationary on a float).
 */
public class ParadeShotCalculator {

    // Tag 22 is a TRENCH tag — center is 35 inches off the floor.
    // Turret exit point is ~21.6 inches off the floor (inherited from ShotCalculator).
    // Tune via SmartDashboard if needed.
    private static final Distance DEFAULT_TAG_22_HEIGHT = Inches.of(35.0);
    private static final Distance DEFAULT_TURRET_HEIGHT = Inches.of(21.6);

    private static final double RATIO_OVER_MIN_VELOCITY = 1.03;

    private static final Distance SHOOTER_WHEEL_DIAMETER = Inches.of(4);
    private static final Distance SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER.times(Math.PI);
    private static final double COUNTER_WHEEL_RECIPROCAL = 1.75;

    private static final double TURRET_TX_KP = 0.02; // tune: duty cycle per degree of tx error

    // EMA smoothing for hood angle
    private static double HOOD_ALPHA = 0.5;
    private static Angle smoothedHoodAngle = Degrees.of(0.0);
    private static boolean hoodInitialized = false;

    private static double smoothEMA(double newValue, double previousSmoothed, double alpha) {
        return alpha * newValue + (1.0 - alpha) * previousSmoothed;
    }

    /**
     * Returns open-loop duty cycle output to drive the turret toward tag 22.
     * Positive tx = target right of boresight = turret should rotate right
     * (positive output).
     *
     * @param tx txnc from tag 22 rawFiducial, in degrees
     * @return duty cycle in [-1, 1]
     */
    public static double calculateTurretDutyCycle(Angle tx) {
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/TurretP", TURRET_TX_KP);
        double kP = SmartDashboard.getNumber("frc3620/ParadeShotCalculator/TurretKp", TURRET_TX_KP);
        double output = MathUtil.clamp(tx.in(Degrees) * kP, -0.05, 0.05);
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/TxDeg", tx.in(Degrees));
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/TurretDutyCycle", output);
        return output;
    }

    /**
     * Calculates the required shooter speed given the measured distance to tag 22.
     *
     * @param distanceToTag distToRobot from tag 22 rawFiducial
     * @return commanded shooter angular velocity
     */
    public static AngularVelocity calculateShooterSpeed(Distance distanceToTag) {
        double ratio = SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/RatioOverMinVelocity", RATIO_OVER_MIN_VELOCITY);

        // Allow SmartDashboard overrides; fall back to defaults
        Distance tagHeight = Inches.of(SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/Tag22HeightInches", DEFAULT_TAG_22_HEIGHT.in(Inches)));
        Distance turretHeight = Inches.of(SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/TurretHeightInches", DEFAULT_TURRET_HEIGHT.in(Inches)));

        Distance hDistance = distanceToTag;
        Distance deltaZ = tagHeight.minus(turretHeight);

        // Minimum-energy exit velocity (same formula as
        // ShotCalculator.calculateBaseExitVelocity)
        double hFt = hDistance.in(Feet);
        double dzFt = deltaZ.in(Feet);
        LinearVelocity exitVelocity = FeetPerSecond.of(
                ratio * Math.sqrt(32.2 * (dzFt + Math.sqrt(hFt * hFt + dzFt * dzFt))));

        // exit velocity → shooter motor RPM (same chain as
        // ShotCalculator.calculateShooterSpeed)
        AngularVelocity rpsBig = RevolutionsPerSecond
                .of(exitVelocity.in(FeetPerSecond) / SHOOTER_WHEEL_CIRCUMFERENCE.in(Feet));
        AngularVelocity shooterRps = RevolutionsPerSecond
                .of(2.0 * rpsBig.in(RevolutionsPerSecond) / COUNTER_WHEEL_RECIPROCAL);

        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/HDistanceFt", hFt);
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/DeltaZFt", dzFt);
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/ExitVelocityFtps", exitVelocity.in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/ShooterRPM", shooterRps.in(RPM));

        return shooterRps.times(0.9);
    }

    /**
     * Calculates the hood angle given the measured distance to tag 22.
     * Uses the theoretical high-arc exit angle. Applies EMA smoothing.
     *
     * @param distanceToTag distToRobot from tag 22 rawFiducial
     * @return hood angle setpoint
     */
    public static Angle calculateHoodAngle(Distance distanceToTag) {
        double ratio = SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/RatioOverMinVelocity", RATIO_OVER_MIN_VELOCITY);
        double ratioAngle = SmartDashboard.getNumber("frc3620/ShooterHood/multiplier", 1.0);

        Distance tagHeight = Inches.of(SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/Tag22HeightInches", DEFAULT_TAG_22_HEIGHT.in(Inches)));
        Distance turretHeight = Inches.of(SmartDashboard.getNumber(
                "frc3620/ParadeShotCalculator/TurretHeightInches", DEFAULT_TURRET_HEIGHT.in(Inches)));

        Distance deltaZ = tagHeight.minus(turretHeight);

        double hFt = distanceToTag.in(Feet);
        double dzFt = deltaZ.in(Feet);

        double exitVelocityFtps = ratio * Math.sqrt(32.2 * (dzFt + Math.sqrt(hFt * hFt + dzFt * dzFt)));
        double v2 = exitVelocityFtps * exitVelocityFtps;

        // High-arc exit angle (same formula as
        // ShotCalculator.calculateHighBaseExitAngle)
        double discriminant = Math.max(
                v2 * v2 - 32.2 * (32.2 * hFt * hFt + 2.0 * dzFt * v2),
                0.0);
        Angle exitAngle = Radians.of(
                Math.atan2(v2 + Math.sqrt(discriminant), 32.2 * hFt))
                .times(ratioAngle);

        Angle rawHoodAngle = Degrees.of(100).minus(exitAngle);

        HOOD_ALPHA = SmartDashboard.getNumber("frc3620/ParadeShotCalculator/HoodAlpha", HOOD_ALPHA);

        if (!hoodInitialized) {
            smoothedHoodAngle = rawHoodAngle;
            hoodInitialized = true;
        }

        smoothedHoodAngle = Degrees.of(
                smoothEMA(rawHoodAngle.in(Degrees), smoothedHoodAngle.in(Degrees), HOOD_ALPHA));

        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/ExitAngleDeg", exitAngle.in(Degrees));
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/HoodAngleRawDeg", rawHoodAngle.in(Degrees));
        SmartDashboard.putNumber("frc3620/ParadeShotCalculator/HoodAngleSmoothDeg", smoothedHoodAngle.in(Degrees));

        return smoothedHoodAngle;
    }

    public static void resetSmoothing() {
        hoodInitialized = false;
    }
}