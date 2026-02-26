package frc.robot.Helpers;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

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

    private static final double ratioOverMinVelocity = 1.05; // placeholder value
    private static final Translation3d turretOffset = new Translation3d(
        Inches.of(0.0), Inches.of(0), Inches.of(0.0));

    public static Distance calculateBaseHDistanceToTarget(Translation2d targetPosition, Pose2d robotPose) {
        Translation2d turretPosition = robotPose.getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.getRotation()));
        Distance distance = Meters.of(targetPosition.getDistance(turretPosition));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHDistanceFt", distance.in(Feet));
        return distance;
    }

    public static Rotation2d calculateBaseFieldAngleToTarget(Translation2d targetPosition, Pose2d robotPose) {
        Translation2d turretPosition = robotPose.getTranslation().plus(turretOffset.toTranslation2d().rotateBy(robotPose.getRotation()));
        Translation2d delta = targetPosition.minus(turretPosition);
        Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY());

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseFieldAngleDeg", rotation.getDegrees());
        return rotation;
    }

    public static Rotation2d calculateBaseTurretAngleToTarget(Translation2d targetPosition, Pose2d robotPose) {
        Rotation2d fieldAngle = calculateBaseFieldAngleToTarget(targetPosition, robotPose);
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d rotation = fieldAngle.minus(robotHeading);

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseTurretAngleDeg", rotation.getDegrees());
        return rotation;
    }

    public static LinearVelocity calculateBaseExitVelocity(Translation3d targetPosition, Pose2d robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());

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

    public static Angle calculateLowBaseExitAngle(Translation3d targetPosition, Pose2d robotPose) {
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
    }

    public static Angle calculateHighBaseExitAngle(Translation3d targetPosition, Pose2d robotPose) {
        Distance hDistance = calculateBaseHDistanceToTarget(targetPosition.toTranslation2d(), robotPose);
        Distance deltaZ = targetPosition.getMeasureZ().minus(turretOffset.getMeasureZ());
        LinearVelocity exitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);

        Angle highExitAngle = Radians.of(Math.atan2(
            (Math.pow(exitVelocity.in(FeetPerSecond), 2) + // Note the plus sign here, adding discriminant creates high arc solution
            Math.max(Math.sqrt(
                Math.pow(exitVelocity.in(FeetPerSecond), 4) - 
                32.2 * (32.2 * Math.pow(hDistance.in(Feet), 2) + 2 * deltaZ.in(Feet) * Math.pow(exitVelocity.in(FeetPerSecond), 2))
            ),0.0)) , (32.2 * hDistance.in(Feet))
        ));

        SmartDashboard.putNumber("frc3620/ShotCalculator/BaseHighExitAngleDeg", highExitAngle.in(Degrees));
        return highExitAngle;
    }

    /**  
    * @param kinematics The SwerveDriveKinematics object used to convert between chassis speeds and module states.
    * @param swerveStates A Supplier that provides the current SwerveDriveState, which contains the states of all swerve modules.
    * @param pigeoYaw The current yaw angle of the robot, used to convert from robot-relative to field-relative speeds.
    */
    public static Translation2d calculateRobotVelocity(SwerveDriveKinematics kinematics,
            SwerveDriveState swerveStates, Rotation2d pigeoYaw) {

        // Gets the robot-relative speeds of the robot based on the current module
        // states
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(swerveStates.ModuleStates);

        // Converts the robot-relative speeds to field-relative speeds using the current robot orientation
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds
                .fromRobotRelativeSpeeds(robotRelativeSpeeds, pigeoYaw);

        Translation2d velocity = new Translation2d(
            Feet.convertFrom(fieldRelativeSpeeds.vxMetersPerSecond, Meters), 
            Feet.convertFrom(fieldRelativeSpeeds.vyMetersPerSecond, Meters));

        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityFtps", velocity.getNorm());
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityXFtps", velocity.getMeasureX().in(Feet));
        SmartDashboard.putNumber("frc3620/ShotCalculator/RobotVelocityYFtps", velocity.getMeasureY().in(Feet));
        return velocity;
    }

    public static Rotation2d calculateFieldAngle(Translation3d targetPosition, 
        Pose2d robotPose, Translation2d robotVelocity) {

        Rotation2d rotation = calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity).getAngle();

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetFieldAngleDeg", rotation.getDegrees());
        return rotation;
    }

    public static Rotation2d calculateNetTurretAngleToTarget(Translation2d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        Rotation2d fieldAngle = calculateFieldAngle(new Translation3d(targetPosition.getMeasureX(), targetPosition.getMeasureY(), Feet.zero()), robotPose, robotVelocity);
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d rotation = fieldAngle.minus(robotHeading);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetTurretAngleDeg", rotation.getDegrees());
        return rotation;
    }

    public static Angle calculateExitAngle(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        Translation2d netHorizontalVelocity = calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity);
        Translation3d netVerticalVelocity = calculateNetVerticalShotVelocity(targetPosition, robotPose, robotVelocity);

        Angle angle = Radians.of(Math.atan2(netVerticalVelocity.getZ(), netHorizontalVelocity.getNorm()));

        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalExitAngleDeg", angle.in(Degrees));
        return angle;
    }

    public static Translation2d calculateNetHorizontalShotVelocity(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        LinearVelocity bExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose);
        Rotation2d bFieldAngle = calculateBaseFieldAngleToTarget(targetPosition.toTranslation2d(), robotPose);

        LinearVelocity bHorizontalExitVelocity = bExitVelocity.times(Math.cos(calculateLowBaseExitAngle(targetPosition, robotPose).in(Radians)));

        double hXExitVelocity = bHorizontalExitVelocity.in(FeetPerSecond) * Math.cos(bFieldAngle.getRadians());
        double hXRobotVelocity = robotVelocity.getMeasureX().in(Feet);

        double hYExitVelocity = bHorizontalExitVelocity.in(FeetPerSecond) * Math.sin(bFieldAngle.getRadians());
        double hYRobotVelocity = robotVelocity.getMeasureY().in(Feet);

        Translation2d net = new Translation2d(hXExitVelocity + hXRobotVelocity, hYExitVelocity + hYRobotVelocity);
        
        SmartDashboard.putNumber("frc3620/ShotCalculator/NetHorizontalShotVX", net.getMeasureX().in(Feet));
        SmartDashboard.putNumber("frc3620/ShotCalculator/NetHorizontalShotVY", net.getMeasureY().in(Feet));

        return net;
    }

    public static Translation3d calculateNetVerticalShotVelocity(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        double verticalExitVelocity = calculateBaseExitVelocity(targetPosition, robotPose).in(FeetPerSecond)
         * Math.sin(calculateLowBaseExitAngle(targetPosition, robotPose).in(Radians));
        double verticalRobotVelocity = 0; // Assuming the robot's velocity is purely horizontal

        Translation3d verticalVelocity = new Translation3d(0, 0, verticalExitVelocity + verticalRobotVelocity);

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetVerticalShotVZ", verticalVelocity.getMeasureZ().in(Feet));
        return verticalVelocity;
    }

    public static Translation3d calculateNetShotVelocity(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        Translation2d netHorizontal = calculateNetHorizontalShotVelocity(targetPosition, robotPose, robotVelocity);
        Translation3d netVertical = calculateNetVerticalShotVelocity(targetPosition, robotPose, robotVelocity);

        Translation3d netShot = new Translation3d(netHorizontal.getX(), netHorizontal.getY(), netVertical.getZ());

        SmartDashboard.putNumber("frc3620/ShotCalculator/NetShotMagnitude", netShot.getNorm());
        return netShot;
    }

    public static Angle calculateHoodAngle(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        Angle hoodAngle = Degrees.of(100).minus(calculateExitAngle(targetPosition, robotPose, robotVelocity));
        
        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalHoodAngle", hoodAngle.in(Degrees));
        return hoodAngle;
    }

    public static AngularVelocity calculateShooterSpeed(Translation3d targetPosition, Pose2d robotPose, Translation2d robotVelocity) {
        Distance wheelCircumference = Inches.of(4).times(Math.PI);
        double counterWheelRatio = 1.75;

        AngularVelocity rpsBig = RevolutionsPerSecond.of(calculateNetShotVelocity(targetPosition, robotPose, robotVelocity).getNorm()/wheelCircumference.in(Feet));
        AngularVelocity rpsSmall = RevolutionsPerSecond.of(rpsBig.magnitude()/counterWheelRatio);

        AngularVelocity rpm = (rpsBig.plus(rpsSmall).div(2)).times(60);

        SmartDashboard.putNumber("frc3620/ShotCalculator/FinalShooterRPM", rpm.in(RPM));

        return rpm;
    }
}
