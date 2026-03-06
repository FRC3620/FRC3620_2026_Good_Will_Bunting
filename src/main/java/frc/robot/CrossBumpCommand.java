// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrossBumpCommand extends Command {

  private final SwerveSubsystem swerve;

  private final double vx;
  private final double vy;
  private final double vRot;

  SwerveRequest.FieldCentric executeRequest = new SwerveRequest.FieldCentric();
  SwerveRequest.FieldCentric endRequest = new SwerveRequest.FieldCentric();

  FieldCentric request = new FieldCentric();
  private final Timer timer = new Timer();

  /** Creates a new CrossBumpCommand. */
  public CrossBumpCommand(SwerveSubsystem swerve, double vx, double vy, double vRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.vx = vx;
    this.vy = vy;
    this.vRot = vRot;

    // should override pathplanner, need to check
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerve.applyRequest(() -> executeRequest
        .withVelocityX(vx)
        .withVelocityY(vy)
        //.withRotationalRate(vRot)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

    swerve.applyRequest(() -> request.withChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx,vy, vRot, Rotation2d.fromDegrees(swerve.getPigeon2().getYaw().getValueAsDouble()))));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerve.applyRequest(() -> endRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (timer.hasElapsed(100000000) && Math.abs(swerve.getPigeon2().getPitch().getValueAsDouble()) < 5);
  }
}
