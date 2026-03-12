// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Subsystems.SwerveSubsystem;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrossBumpCommand extends Command {

  private final SwerveSubsystem swerve;

  private double vx;
  private double vy;
  private double vRot;

  SwerveRequest.FieldCentric drive;
  SwerveRequest.FieldCentric.ApplyFieldSpeeds speeds = new SwerveRequest.FieldCentric.ApplyFieldSpeeds();
  ChassisSpeeds cSpeeds = new ChassisSpeeds();
  ChassisSpeeds endSpeeds = new ChassisSpeeds(0, 0, 0);

  public static Alliance color;

  private final Timer timer = new Timer();

  /** Creates a new CrossBumpCommand. */
  public CrossBumpCommand(SwerveSubsystem swerve, SwerveRequest.FieldCentric drive, double vx, double vy, double vRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.drive = drive;
    this.vx = vx;
    this.vy = vy;
    this.vRot = vRot;

    // should override pathplanner, need to check
    //addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    color = DriverStation.getAlliance().get();

    if (color == Alliance.Blue) {
      vx = -vx;
    }

    cSpeeds = new ChassisSpeeds(vx, vy, vRot);

    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerve.setControl(speeds.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(cSpeeds, new Rotation2d())));
    /*
     * swerve.applyRequest(() -> drive
     * .withVelocityX(vx)
     * .withVelocityY(vy)
     * .withRotationalRate(vRot)
     * .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
     */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    /*
     * swerve.applyRequest(() -> drive
     * .withVelocityX(0)
     * .withVelocityY(0)
     * .withRotationalRate(0)
     * .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
     */

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (timer.hasElapsed(.25) && Math.abs(swerve.getPigeon2().getPitch().getValueAsDouble()) < 5);
  }
}
