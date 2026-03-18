package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotCalculator {

    public enum FieldTargets {
        //EXACT CENTER OF HUB

        BLUE_HUB(new Translation3d(Inches.of(182.11), Inches.of(158.84), Inches.of(72))),
        //TBD
        OP_PASS(new Translation3d(Inches.of(150), Inches.of(98.85), Inches.of(0))),
        DEPOT_PASS(new Translation3d(Inches.of(150), Inches.of(218.838), Inches.of(0)));     
        
        private final Translation3d targetPosition;

        FieldTargets(Translation3d targetPosition) {
            this.targetPosition = targetPosition;
        }

        public Translation3d getTargetPosition() {
            return targetPosition;
        }
    }

    private static final double ratioOverMinVelocity = 1.03; // placeholder value
    private static final Translation3d turretOffset = new Translation3d(
        Meters.of(-0.152), Meters.of(0.122), Meters.of(0.54864));

    private static AngularVelocity shooterSpeed = RotationsPerSecond.of(0);
    private static double preShooterRatio = 1.0;

    public static Distance calculateBaseHDistanceToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
        Distance distance = Meters.of(targetPosition.getDistance(turretPosition));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHDistanceFt", distance.in(Feet));
        return distance;
    }

    public static Angle calculateBaseFieldAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
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
        double ratioOverMinVelocity = SmartDashboard.getNumber("frc3620/ShotCalculator/Ratio Over Min Velocity", 1.03); // empirically determined ratio to add to the minimum velocity to ensure the shot reaches the target, accounts for drag and other unmodeled factors

        LinearVelocity exitVelocity = FeetPerSecond.of(
            ratioOverMinVelocity
             * Math.sqrt(
                (32.2 * (deltaZ.in(Feet) + Math.sqrt(
                        Math.pow(hDistance.in(Feet), 2) + Math.pow(deltaZ.in(Feet), 2))))
            )
        );

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseExitVelocityFtps", exitVelocity.in(FeetPerSecond));
        return exitVelocity;
    }

/*     public static Angle calculateLowBaseExitAngle(Translation3d targetPosition, Pose2d robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);

        Angle lowExitAngle = Radians.of(Math.atan2(
            (Math.pow(exitVelocity.in(FeetPerSecond), 2) - // note the minus sign here, subtracting the discriminant gives the low angle solution
            Math.max(Math.sqrt(
                Math.pow(exitVelocity.in(FeetPerSecond), 4) - 
                32.2 * (32.2 * Math.pow(hDistance.in(Feet), 2) + 2 * deltaZ.in(Feet) * Math.pow(exitVelocity.in(FeetPerSecond), 2))
            ),0.0)) , (32.2 * hDistance.in(Feet))
        ));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseLowExitAngleDeg", lowExitAngle.in(Degrees));
        return lowExitAngle;
    } */

    public static Angle calculateHighBaseExitAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);
        double ratioAngle = SmartDashboard.getNumber("frc3620/ShooterHood/multiplier", 1);

        Angle highExitAngle = Radians.of(
            Math.atan2(
                exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)
                + Math.sqrt(Math.max(
                    exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)
                    - 32.2 * ((32.2 * hDistance.in(Feet) * hDistance.in(Feet)) + 2 * deltaZ.in(Feet) * exitVelocity.in(FeetPerSecond) * exitVelocity.in(FeetPerSecond)), 0.0))
                    , (32.2 * hDistance.in(Feet))
                )
            ).times(ratioAngle);

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHighExitAngleDeg", highExitAngle.in(Degrees));
        return highExitAngle;
    }

    /**  
    * @param kinematics The SwerveDriveKinematics object used to convert between chassis speeds and module states.
    * @param swerveStates A Supplier that provides the current SwerveDriveState, which contains the states of all swerve modules.
    * @param pigeoYaw The current yaw angle of the robot, used to convert from robot-relative to field-relative speeds.
    */
    public static VelocityVector calculateRobotVelocity(SwerveDriveKinematics kinematics,
            SwerveDriveState swerveStates, Rotation2d pigeoYaw) {

                double xMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/XVelocityMultipler", 1);
                double yMultiplier = SmartDashboard.getNumber("frc3620/ShotCalculator/YVelocityMultipler", 1);
        // Gets the robot-relative speeds of the robot based on the current module
        // states
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(swerveStates.ModuleStates);

        // Converts the robot-relative speeds to field-relative speeds using the current robot orientation
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

    public static Angle calculateFieldAngle(Translation3d targetPosition, 
        Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {

        VelocityVector netHorizontalVelocity = calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
        Angle rotation = Radians.of(Math.atan2(netHorizontalVelocity.getY().in(FeetPerSecond), netHorizontalVelocity.getX().in(FeetPerSecond)));

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetFieldAngleDeg", rotation.in(Degrees));
        return rotation;
    }

    public static Angle calculateNetTurretAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        Angle fieldAngle = calculateFieldAngle(new Translation3d(targetPosition.getMeasureX(), targetPosition.getMeasureY(), Feet.zero()), robotPose, robotVelocity);
        Angle robotHeading = robotPose.get().getRotation().getMeasure();
        Angle rotation = fieldAngle.minus(robotHeading);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetTurretAngleDeg", rotation.in(Degrees));
        return rotation;
    }

    public static Angle calculateExitAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        VelocityVector netHorizontalVelocity = calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
        LinearVelocity netVerticalVelocity = calculateNetVerticalVelocity(targetPosition, robotPose, robotVelocity);

        Angle angle = Radians.of(Math.atan2(netVerticalVelocity.in(FeetPerSecond), netHorizontalVelocity.getNorm().in(FeetPerSecond)));

        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalExitAngleDeg", angle.in(Degrees));
        return angle;
    }

    public static VelocityVector calculateNetHorizontalVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        LinearVelocity bExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);
        Angle bFieldAngle = calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose);

        LinearVelocity bHorizontalExitVelocity = bExitVelocity.times(Math.cos(calculateHighBaseExitAngle(targetPosition, robotPose).in(Radians)));

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

    public static LinearVelocity calculateNetVerticalVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        LinearVelocity verticalExitVelocity = FeetPerSecond.of(calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond) * Math.sin(calculateHighBaseExitAngle(targetPosition, robotPose).in(Radians)));
        LinearVelocity verticalRobotVelocity = FeetPerSecond.of(0); // Assuming the robot's velocity is purely horizontal

        LinearVelocity verticalVelocity = verticalExitVelocity.plus(verticalRobotVelocity);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetVerticalShotVZ", verticalVelocity.in(FeetPerSecond));
        return verticalVelocity;
    }

    public static LinearVelocity calculateNetShotVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        VelocityVector netHorizontal = calculateNetHorizontalVelocity(targetPosition, robotPose, robotVelocity);
        LinearVelocity netVertical = calculateNetVerticalVelocity(targetPosition, robotPose, robotVelocity);

        VelocityVector netShot = new VelocityVector(netHorizontal.getNorm(), netVertical);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetShotMagnitude", netShot.getNorm().in(FeetPerSecond));
        return netShot.getNorm();
    }

    public static Angle calculateHoodAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
        Angle hoodAngle = Degrees.of(100).minus(calculateExitAngle(targetPosition, robotPose, robotVelocity));
        
        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalHoodAngle", hoodAngle.in(Degrees));
        return hoodAngle;
    }


    public static AngularVelocity calculateShooterSpeed(
            Translation3d targetPosition,
            Supplier<Pose2d> robotPose,
            Supplier<VelocityVector> robotVelocity) {

        LinearVelocity shotSpeed = calculateNetShotVelocity(targetPosition, robotPose, robotVelocity);

        Distance wheelDiameter = Inches.of(4);
        Distance wheelCircumference =
                wheelDiameter.times(Math.PI);

        double counterWeelRecibrocahl = 1.75;

        AngularVelocity rpsBig = RevolutionsPerSecond.of(shotSpeed.in(FeetPerSecond) / wheelCircumference.in(Feet));
        AngularVelocity shooterRps = RevolutionsPerSecond.of(2 * rpsBig.in(RevolutionsPerSecond) / counterWeelRecibrocahl);

        
        shooterSpeed = shooterRps;

        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalShooterRPM", shooterRps.in(RPM));
        return shooterRps;
    }

    public static AngularVelocity calculatePreshooterSpeed() {
        preShooterRatio = SmartDashboard.getNumber("frc3620/ShotCalculator/PreShooterRatio", preShooterRatio);
        return shooterSpeed.times(preShooterRatio);
    }
}
