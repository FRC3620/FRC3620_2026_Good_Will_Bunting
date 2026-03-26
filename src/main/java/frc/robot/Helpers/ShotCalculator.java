package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Horsepower;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotCalculator {

    private static final NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable("dashboard")
            .getSubTable("target");

    private static final DoubleArraySubscriber hubSub = dashboardTable.getDoubleArrayTopic("hub")
            .subscribe(new double[] {
                    (Inches.of(182.11)).in(Meters),
                    (Inches.of(158.84)).in(Meters),
                    (Inches.of(72)).in(Feet) // z is in feet
            });

    private static final DoubleArraySubscriber opPassSub = dashboardTable.getDoubleArrayTopic("op_pass")
            .subscribe(new double[] {
                    (Inches.of(126)).in(Meters),
                    (Inches.of(98.85)).in(Meters),
                    0
            });

    private static final DoubleArraySubscriber depotPassSub = dashboardTable.getDoubleArrayTopic("depot_pass")
            .subscribe(new double[] {
                    (Inches.of(126)).in(Meters),
                    (Inches.of(218.838)).in(Meters),
                    0
            });

    public enum FieldTargets {
        BLUE_HUB(new Translation3d(Inches.of(182.11), Inches.of(158.84), Inches.of(72))),
        OP_PASS(new Translation3d(Inches.of(126), Inches.of(98.85), Inches.of(0))),
        DEPOT_PASS(new Translation3d(Inches.of(126), Inches.of(218.838), Inches.of(0)));

        private Translation3d targetPosition;

        FieldTargets(Translation3d targetPosition) {
            this.targetPosition = targetPosition;
        }

        public Translation3d getTargetPosition() {
            return targetPosition;
        }

        // Allow dashboard to override any target
        public void setTargetPosition(Translation3d position) {
            this.targetPosition = position;
        }
    }

    private static final double ratioOverMinVelocity = 1.03; // placeholder value
    private static final Translation3d turretOffset = new Translation3d(
            Meters.of(-0.152), Meters.of(0.122), Meters.of(0.54864));

    private static AngularVelocity shooterSpeed = RotationsPerSecond.of(0);
    private static double preShooterRatio = 1.0;

    private static final Distance SHOOTER_WHEEL_DIAMETER = Inches.of(4);
    private static final Distance SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER.times(Math.PI);
    private static final double COUNTER_WHEEL_RECIPROCAL = 1.75;

    // Smoothing constants - tune these, higher = more responsive, lower = smoother
    private static double HOOD_ALPHA = 0.15;
    private static double TURRET_ALPHA = 0.1; // turret is heavier, smooth it more

    // Previous smoothed values
    private static Angle smoothedHoodAngle = Degrees.of(0.0);
    private static Angle smoothedTurretAngle = Degrees.of(0.0);
    private static boolean hoodInitialized = false;
    private static boolean turretInitialized = false;

    private static double smoothEMA(double newValue, double previousSmoothed, double alpha) {
        return alpha * newValue + (1.0 - alpha) * previousSmoothed;
    }

    public static void updateFromDashboard() {
        syncTarget(hubSub, FieldTargets.BLUE_HUB, "Hub");
        syncTarget(opPassSub, FieldTargets.OP_PASS, "OpPass");
        syncTarget(depotPassSub, FieldTargets.DEPOT_PASS, "DepotPass");
    }

    private static void syncTarget(DoubleArraySubscriber sub, FieldTargets target, String name) {
        double[] pose = sub.get();
        if (pose.length >= 3) {
            target.setTargetPosition(new Translation3d(
                    Meters.of(pose[0]),
                    Meters.of(pose[1]),
                    Feet.of(pose[2]) // z comes in as feet
            ));
            SmartDashboard.putNumber("frc3620/ShotCalculator/" + name + "X", pose[0]);
            SmartDashboard.putNumber("frc3620/ShotCalculator/" + name + "Y", pose[1]);
            SmartDashboard.putNumber("frc3620/ShotCalculator/" + name + "ZFt", pose[2]);
        }
    }

    public static Distance calculateBaseHDistanceToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation()
                .plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
        Distance distance = Meters.of(targetPosition.getDistance(turretPosition));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHDistanceFt", distance.in(Feet));
        return distance;
    }

    public static Angle calculateBaseFieldAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation()
                .plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
        Translation2d delta = targetPosition.minus(turretPosition);
        Angle rotation = Radians.of(Math.atan2(delta.getY(), delta.getX()));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseFieldAngleDeg", rotation.in(Degrees));
        return rotation;
    }

    public static Angle calculateBaseTurretAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Angle fieldAngle = calculateBaseFieldAngleToTarget(targetPosition, robotPose);
        Angle robotHeading = robotPose.get().getRotation().getMeasure();
        Angle rotation = fieldAngle.minus(robotHeading);

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseTurretAngleDeg", rotation.in(Degrees));
        return rotation;
    }

    public static LinearVelocity calculateBaseExitVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        double ratio = SmartDashboard.getNumber("frc3620/ShotCalculator/Ratio Over Min Velocity",
                ratioOverMinVelocity); // empirically
        // determined
        // ratio
        // to
        // add
        // to
        // the
        // minimum
        // velocity
        // to
        // ensure
        // the
        // shot
        // reaches
        // the
        // target,
        // accounts
        // for
        // drag
        // and
        // other
        // unmodeled
        // factors

        LinearVelocity exitVelocity = FeetPerSecond.of(
                ratio
                        * Math.sqrt(
                                (32.2 * (deltaZ.in(Feet) + Math.sqrt(
                                        Math.pow(hDistance.in(Feet), 2) + Math.pow(deltaZ.in(Feet), 2))))));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseExitVelocityFtps", exitVelocity.in(FeetPerSecond));
        return exitVelocity;
    }

    /*
     * public static Angle calculateLowBaseExitAngle(Translation3d targetPosition,
     * Pose2d robotPose) {
     * Distance hDistance =
     * calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
     * Distance deltaZ =
     * targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
     * LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition,
     * robotPose);
     * 
     * Angle lowExitAngle = Radians.of(Math.atan2(
     * (Math.pow(exitVelocity.in(FeetPerSecond), 2) - // note the minus sign here,
     * subtracting the discriminant gives the low angle solution
     * Math.max(Math.sqrt(
     * Math.pow(exitVelocity.in(FeetPerSecond), 4) -
     * 32.2 * (32.2 * Math.pow(hDistance.in(Feet), 2) + 2 * deltaZ.in(Feet) *
     * Math.pow(exitVelocity.in(FeetPerSecond), 2))
     * ),0.0)) , (32.2 * hDistance.in(Feet))
     * ));
     * 
     * SmartDashboard.putNumber("frc3620/ShotCalculator/BaseLowExitAngleDeg",
     * lowExitAngle.in(Degrees));
     * return lowExitAngle;
     * }
     */

    public static Angle calculateHighBaseExitAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);
        double ratioAngle = SmartDashboard.getNumber("frc3620/ShooterHood/multiplier", 1);

        Angle highExitAngle = Radians.of(
                Math.atan2(
                        exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)
                                + Math.sqrt(Math.max(
                                        exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)
                                                * exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)
                                                - 32.2 * ((32.2 * hDistance.in(Feet) * hDistance.in(Feet))
                                                        + 2 * deltaZ.in(Feet) * exitVelocity.in(FeetPerSecond)
                                                                * exitVelocity.in(FeetPerSecond)),
                                        0.0)),
                        (32.2 * hDistance.in(Feet))))
                .times(ratioAngle);

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHighExitAngleDeg", highExitAngle.in(Degrees));
        return highExitAngle;
    }

    /**
     * @param kinematics   The SwerveDriveKinematics object used to convert between
     *                     chassis speeds and module states.
     * @param swerveStates A Supplier that provides the current SwerveDriveState,
     *                     which contains the states of all swerve modules.
     * @param pigeoYaw     The current yaw angle of the robot, used to convert from
     *                     robot-relative to field-relative speeds.
     */
    public static VelocityVector calculateRobotVelocity(SwerveDriveKinematics kinematics,
            SwerveDriveState swerveStates, Rotation2d pigeoYaw) {

        double xMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/XVelocityMultipler", 1);
        double yMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/YVelocityMultipler", 1);
        // Gets the robot-relative speeds of the robot based on the current module
        // states
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(swerveStates.ModuleStates);

        // Converts the robot-relative speeds to field-relative speeds using the current
        // robot orientation
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds
                .fromRobotRelativeSpeeds(robotRelativeSpeeds, pigeoYaw);

        VelocityVector velocity = new VelocityVector(
                FeetPerSecond.of(Feet.convertFrom(fieldRelativeSpeeds.vxMetersPerSecond, Meters)).times(xMultiplier),
                FeetPerSecond.of(Feet.convertFrom(fieldRelativeSpeeds.vyMetersPerSecond, Meters)).times(yMultiplier));

        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityFtps", velocity.getNorm().in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityXFtps", velocity.getX().in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityYFtps", velocity.getY().in(FeetPerSecond));
        return velocity;
    }

    public static VelocityVector calculateQuestVelocity(LinearVelocity questVX, LinearVelocity questVY,
            Rotation2d questYaw) {

        double xMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/XVelocityMultipler", 1);
        double yMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/YVelocityMultipler", 1);

        VelocityVector velocity = new VelocityVector(questVX.times(xMultiplier), questVY.times(yMultiplier));

        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityFtps", velocity.getNorm().in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityXFtps", velocity.getX().in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityYFtps", velocity.getY().in(FeetPerSecond));

        return velocity;
    }

    public static Angle calculateFieldAngle(Translation3d targetPosition,
            Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {

        VelocityVector netHorizontalVelocity = calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
        Angle rotation = Radians.of(Math.atan2(netHorizontalVelocity.getY().in(FeetPerSecond),
                netHorizontalVelocity.getX().in(FeetPerSecond)));

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetFieldAngleDeg", rotation.in(Degrees));
        return rotation;
    }

    public static Angle calculateNetTurretAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity) {
        Angle fieldAngle = calculateFieldAngle(
                new Translation3d(targetPosition.getMeasureX(), targetPosition.getMeasureY(), Feet.zero()), robotPose,
                robotVelocity);
        Angle robotHeading = robotPose.get().getRotation().getMeasure();
        Angle rotation = fieldAngle.minus(robotHeading);

        if (!turretInitialized) {
            smoothedTurretAngle = rotation;
            turretInitialized = true;
        }

        TURRET_ALPHA = SmartDashboard.getNumber("frc3620/ShotCalculator/TurretAlpha", TURRET_ALPHA);

        Angle newAngle = rotation;
        Angle lastAngle = smoothedTurretAngle;

        double delta = MathUtil.inputModulus(newAngle.minus(lastAngle).in(Degrees), -180, 180);
        smoothedTurretAngle = lastAngle.plus(Degrees.of(delta * TURRET_ALPHA));

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetTurretAngleDegRaw", rotation.in(Degrees));
        SmartDashboard.putNumber("frc3620/ShotCalculator/NetTurretAngleDegSmooth", smoothedTurretAngle.in(Degrees));
        return smoothedTurretAngle;
    }

    /*
     * public static Angle calculateExitAngle(Translation3d targetPosition,
     * Supplier<Pose2d> robotPose,
     * Supplier<VelocityVector> robotVelocity) {
     * VelocityVector netHorizontalVelocity =
     * calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
     * LinearVelocity netVerticalVelocity =
     * calculateNetVerticalVelocity(targetPosition, robotPose, robotVelocity);
     * 
     * Angle angle = Radians.of(
     * Math.atan2(netVerticalVelocity.in(FeetPerSecond),
     * netHorizontalVelocity.getNorm().in(FeetPerSecond)));
     * 
     * SmartDashboard.putNumber("frc3620/ShotCalculator/FinalExitAngleDeg",
     * angle.in(Degrees));
     * return angle;
     * }
     */

    public static Angle calculateExitAngleFromActualSpeed(
            Translation3d targetPosition,
            Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity,
            Supplier<AngularVelocity> actualShooterSpeed) {

        // Convert actual shooter wheel speed back to linear exit velocity
        // (inverse of calculateShooterSpeed)

        // Reverse the shooter gear math from calculateShooterSpeed
        AngularVelocity rpsBig = RevolutionsPerSecond.of(
                actualShooterSpeed.get().in(RevolutionsPerSecond) * COUNTER_WHEEL_RECIPROCAL / 2.0);
        LinearVelocity actualExitVelocity = FeetPerSecond.of(
                rpsBig.in(RevolutionsPerSecond) * SHOOTER_WHEEL_CIRCUMFERENCE.in(Feet));

        SmartDashboard.putNumber("frc3620/ShotCalculator/ActualExitVelocityFtps",
                actualExitVelocity.in(FeetPerSecond));

        // Recompute net horizontal velocity using actual exit velocity
        Angle bFieldAngle = calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose);
        Angle bExitAngle = calculateHighBaseExitAngle(targetPosition, robotPose);

        LinearVelocity actualHorizontalExitVelocity = actualExitVelocity
                .times(Math.cos(bExitAngle.in(Radians)));

        LinearVelocity hXExitVelocity = actualHorizontalExitVelocity.times(Math.cos(bFieldAngle.in(Radians)));
        LinearVelocity hYExitVelocity = actualHorizontalExitVelocity.times(Math.sin(bFieldAngle.in(Radians)));

        LinearVelocity netHXV = hXExitVelocity.minus(robotVelocity.get().getX());
        LinearVelocity netHYV = hYExitVelocity.minus(robotVelocity.get().getY());
        VelocityVector netHorizontal = new VelocityVector(netHXV, netHYV);

        LinearVelocity actualVerticalExitVelocity = actualExitVelocity
                .times(Math.sin(bExitAngle.in(Radians)));

        Angle angle = Radians.of(Math.atan2(
                actualVerticalExitVelocity.in(FeetPerSecond),
                netHorizontal.getNorm().in(FeetPerSecond)));

        SmartDashboard.putNumber("frc3620/ShotCalculator/ActualExitAngleDeg", angle.in(Degrees));
        return angle;
    }

    public static Angle calculateHoodAngle(
            Translation3d targetPosition,
            Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity,
            Supplier<AngularVelocity> actualShooterSpeed) {

        Angle rawHoodAngle = Degrees.of(100).minus(
                calculateExitAngleFromActualSpeed(targetPosition, robotPose, robotVelocity, actualShooterSpeed));

        if (!hoodInitialized) {
            smoothedHoodAngle = rawHoodAngle;
            hoodInitialized = true;
        }

        HOOD_ALPHA = SmartDashboard.getNumber("frc3620/ShotCalculator/HoodAlpha", HOOD_ALPHA);

        smoothedHoodAngle = Degrees.of(smoothEMA(rawHoodAngle.in(Degrees), smoothedHoodAngle.in(Degrees), HOOD_ALPHA));
        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalHoodAngleFromActualRaw", rawHoodAngle.in(Degrees));
        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalHoodAngleFromActualSmooth",
                smoothedHoodAngle.in(Degrees));
        return smoothedHoodAngle;
    }

    public static VelocityVector calculateNetHorizontalVelocity(Translation3d targetPosition,
            Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        LinearVelocity bExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);
        Angle bFieldAngle = calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose);

        LinearVelocity bHorizontalExitVelocity = bExitVelocity
                .times(Math.cos(calculateHighBaseExitAngle(targetPosition, robotPose).in(Radians)));

        LinearVelocity hXExitVelocity = bHorizontalExitVelocity.times(Math.cos(bFieldAngle.in(Radians)));
        LinearVelocity hYExitVelocity = bHorizontalExitVelocity.times(Math.sin(bFieldAngle.in(Radians)));
        LinearVelocity hXRobotVelocity = robotVelocity.get().getX();
        LinearVelocity hYRobotVelocity = robotVelocity.get().getY();

        LinearVelocity netHXV = hXExitVelocity.minus(hXRobotVelocity);
        LinearVelocity netHYV = hYExitVelocity.minus(hYRobotVelocity);

        VelocityVector net = new VelocityVector(netHXV, netHYV);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetHorizontalShotVX", netHXV.in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/NetHorizontalShotVY", netHYV.in(FeetPerSecond));
        SmartDashboard.putNumber("frc3620/ShotCalculator/NetHorizontalShotV", net.getNorm().in(FeetPerSecond));
        return net;
    }

    public static LinearVelocity calculateNetVerticalVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity) {
        LinearVelocity verticalExitVelocity = FeetPerSecond
                .of(calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond)
                        * Math.sin(calculateHighBaseExitAngle(targetPosition, robotPose).in(Radians)));
        LinearVelocity verticalRobotVelocity = FeetPerSecond.of(0); // Assuming the robot's velocity is purely
                                                                    // horizontal

        LinearVelocity verticalVelocity = verticalExitVelocity.plus(verticalRobotVelocity);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetVerticalShotVZ", verticalVelocity.in(FeetPerSecond));
        return verticalVelocity;
    }

    public static LinearVelocity calculateNetShotVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity) {
        VelocityVector netHorizontal = calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
        LinearVelocity netVertical = calculateNetVerticalVelocity(targetPosition, robotPose, robotVelocity);

        VelocityVector netShot = new VelocityVector(netHorizontal.getNorm(), netVertical);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetShotMagnitude", netShot.getNorm().in(FeetPerSecond));
        return netShot.getNorm();
    }

    /*
     * public static Angle calculateHoodAngle(Translation3d targetPosition,
     * Supplier<Pose2d> robotPose,
     * Supplier<VelocityVector> robotVelocity, Supplier<AngularVelocity>
     * shooterAngularVelocity) {
     * Angle hoodAngle =
     * Degrees.of(100).minus(calculateExitAngleFromActualSpeed(targetPosition,
     * robotPose, robotVelocity, shooterAngularVelocity));
     * 
     * SmartDashboard.putNumber("frc3620/ShotCalculator/FinalHoodAngle",
     * hoodAngle.in(Degrees));
     * return hoodAngle;
     * }
     */

    public static AngularVelocity calculateShooterSpeed(
            Translation3d targetPosition,
            Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity) {

        LinearVelocity shotSpeed = calculateNetShotVelocity(targetPosition, robotPose, robotVelocity);

        AngularVelocity rpsBig = RevolutionsPerSecond
                .of(shotSpeed.in(FeetPerSecond) / SHOOTER_WHEEL_CIRCUMFERENCE.in(Feet));
        AngularVelocity shooterRps = RevolutionsPerSecond
                .of(2 * rpsBig.in(RevolutionsPerSecond) / COUNTER_WHEEL_RECIPROCAL);

        shooterSpeed = shooterRps;

        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalShooterRPM", shooterRps.in(RPM));
        return shooterRps;
    }

    public static AngularVelocity calculatePreshooterSpeed() {
        preShooterRatio = SmartDashboard.getNumber("frc3620/ShotCalculator/PreShooterRatio", preShooterRatio);
        return shooterSpeed.times(preShooterRatio);
    }

    // In ShotCalculator, add a method to publish all shot data at once
    public static void publishShotData(){

    }
}
