package frc.robot.Helpers;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class ShotCalculator {

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

        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

}
