package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.FieldConstants;

public class ShotCalculator {

    private static final double ratioOverMinVelocity = 1.05; // placeholder value
    private static final Translation3d turretOffset = new Translation3d(
        Inches.of(0.0), Inches.of(0), Inches.of(0.0));

    public static Distance calculateBaseHDistanceToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
        return Feet.of(targetPosition.getDistance(turretPosition));
    }

    public static Rotation2d calculateBaseFieldAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Translation2d turretPosition = robotPose.get().getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.get().getRotation()));
        Translation2d delta = AllianceFlipUtil.apply(targetPosition).minus(AllianceFlipUtil.apply(turretPosition));
        return new Rotation2d(delta.getX(), delta.getY());
    }

    public static Rotation2d calculateBaseTurretAngleToTarget(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
        Rotation2d fieldAngle = calculateBaseFieldAngleToTarget(targetPosition, robotPose);
        Rotation2d robotHeading = AllianceFlipUtil.apply(robotPose.get().getRotation());
        return fieldAngle.minus(robotHeading);
    }

    public static LinearVelocity calculateBaseExitVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());

        LinearVelocity exitVelocity = FeetPerSecond.of(
            ratioOverMinVelocity
             * Math.sqrt(
                (32.2 * (deltaZ.in(Feet) + Math.sqrt(
                        Math.pow(hDistance.in(Feet), 2) + Math.pow(deltaZ.in(Feet), 2))))
            )
        );

        return exitVelocity;
    }

    public static Angle calculateLowBaseHoodAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);

        Angle lowHoodAngle = Radians.of(Math.atan(
            (Math.pow(exitVelocity.in(FeetPerSecond), 2) - // note the minus sign here, subtracting the discriminant gives the low angle solution
            Math.max(Math.sqrt(
                Math.pow(exitVelocity.in(FeetPerSecond), 4) - 
                32.2 * (32.2 * Math.pow(hDistance.in(Feet), 2) + 2 * deltaZ.in(Feet) * Math.pow(exitVelocity.in(FeetPerSecond), 2))
            ),0.0)) / (32.2 * hDistance.in(Feet))
        ));

        return lowHoodAngle;
    }

    public static Angle calculateHighBaseHoodAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);

        Angle highHoodAngle = Radians.of(Math.atan(
            (Math.pow(exitVelocity.in(FeetPerSecond), 2) + // Note the plus sign here, adding discriminant creates high arc solution
            Math.max(Math.sqrt(
                Math.pow(exitVelocity.in(FeetPerSecond), 4) - 
                32.2 * (32.2 * Math.pow(hDistance.in(Feet), 2) + 2 * deltaZ.in(Feet) * Math.pow(exitVelocity.in(FeetPerSecond), 2))
            ),0.0)) / (32.2 * hDistance.in(Feet))
        ));

        return highHoodAngle;
    }

    /**  
    * @param kinematics The SwerveDriveKinematics object used to convert between chassis speeds and module states.
    * @param swerveStates A Supplier that provides the current SwerveDriveState, which contains the states of all swerve modules.
    * @param pigeoYaw The current yaw angle of the robot, used to convert from robot-relative to field-relative speeds.
    */
    public static Translation2d calculateRobotVelocity(SwerveDriveKinematics kinematics,
            Supplier<SwerveDriveState> swerveStates, Supplier<Rotation2d> pigeoYaw) {

        // Gets the robot-relative speeds of the robot based on the current module
        // states
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(swerveStates.get().ModuleStates);

        // Converts the robot-relative speeds to field-relative speeds using the current robot orientation
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds
                .fromRobotRelativeSpeeds(robotRelativeSpeeds, pigeoYaw.get());

        Translation2d allianceCorrectedVelocity = AllianceFlipUtil.apply(new Translation2d(
            Feet.convertFrom(fieldRelativeSpeeds.vxMetersPerSecond, Meters), 
            Feet.convertFrom(fieldRelativeSpeeds.vyMetersPerSecond, Meters)));
        return allianceCorrectedVelocity;
    }

    /**
     * Calculates the angle the turret needs to be at to aim at the target, 
     * taking into account the robot's current position, heading, and velocity.
     * @param targetPosition The position of the target on the field, as a Translation2d (x, y) in meters.
     * @param robotPosition The current position of the robot on the field, as a Pose2d.
     * @param robotHeading The current heading of the robot, as a Rotation2d.
     * @param robotVelocity The current velocity of the robot, as a Translation2d.
     * @return The angle the turret needs to be at to hit the target, as a Rotation2d.
     */
    public static Rotation2d calculateTurretAngle(Translation3d targetPosition, 
        Supplier<Pose2d> robotPose, Translation2d robotVelocity) {

        return calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity).getAngle();
    }

    public static Angle calculateHoodAngle(Translation3d targetPosition, Supplier<Pose2d> robotPose, Translation2d robotVelocity) {
        Translation2d netHorizontalVelocity = calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity);
        Translation3d netVerticalVelocity = calculateNetVerticalShotVelocity(targetPosition, robotPose, robotVelocity);

        return Radians.of(Math.atan2(netVerticalVelocity.getZ(), netHorizontalVelocity.getNorm()));
    }

    public static Translation2d calculateNetHorizontalShotVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Translation2d robotVelocity) {
        double hXExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond) * Math.cos(calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose).getRadians());
        double hXRobotVelocity = robotVelocity.getMeasureX().in(Feet);

        double hYExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond) * Math.sin(calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose).getRadians());
        double hYRobotVelocity = robotVelocity.getMeasureY().in(Feet);

        return new Translation2d(hXExitVelocity + hXRobotVelocity, hYExitVelocity + hYRobotVelocity);
    }

    public static Translation3d calculateNetVerticalShotVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Translation2d robotVelocity) {
        double verticalExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond)
         * Math.sin(calculateHighBaseHoodAngle(targetPosition, robotPose).in(Radians));
        double verticalRobotVelocity = 0; // Assuming the robot's velocity is purely horizontal

        return new Translation3d(0, 0, verticalExitVelocity + verticalRobotVelocity);
    }

    public static Translation3d calculateNetShotVelocity(Translation3d targetPosition, Supplier<Pose2d> robotPose, Translation2d robotVelocity) {
        Translation2d netHorizontal = calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity);
        Translation3d netVertical = calculateNetVerticalShotVelocity(targetPosition, robotPose, robotVelocity);
        return new Translation3d(netHorizontal.getX(), netHorizontal.getY(), netVertical.getZ());
    }

}
