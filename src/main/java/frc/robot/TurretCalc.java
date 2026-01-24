// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class TurretCalc {
    public TurretCalc() {
    }

    public static Angle calculateTurretAngle(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        double turretAngle = angleToTarget - robotHeading;

        MathUtil.clamp(turretAngle, -90, 180);

        return Degrees.of(turretAngle);
    }


}
