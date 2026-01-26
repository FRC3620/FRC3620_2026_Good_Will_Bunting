// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class TurretCalc {

    public enum Targets {
        REDHUB(Inches.of(182.11), Inches.of(158.84)),
        BLUEHUB(Inches.of(4.06), Inches.of(4.0)),
        REDTOPPASS(Inches.of(4.06), Inches.of(4.0)),
        REDBOTTOMPASS(Inches.of(4.06), Inches.of(4.0)),
        BLUETOPPASS(Inches.of(4.06), Inches.of(4.0)),
        BLUEBOTTOMPASS(Inches.of(4.06), Inches.of(4.0));

        public final Translation2d position;

        private Targets(Distance x, Distance y) {
            this.position = new Translation2d(x, y);
        }
    }

    public TurretCalc() {
    }

    public static Angle calculateTurretAngle(Pose2d robotPose, Translation2d targetPose) {

        Rotation2d angleToTarget = new Translation2d(
                targetPose.getX() - robotPose.getX(),
                targetPose.getY() - robotPose.getY()).getAngle();

        Rotation2d turretAngle = angleToTarget.minus(robotPose.getRotation());

        return Radians.of(turretAngle.getRadians());

    }

}
