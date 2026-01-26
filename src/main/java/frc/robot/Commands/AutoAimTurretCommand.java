// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.TurretCalc;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimTurretCommand extends Command {
  /** Creates a new AutoAimTurretCommand. */
  private final TurretSubsystem turret;
  private final SwerveSubsystem swerve;

  private final Optional<Alliance> alliance;

  Translation2d hubPose = new Translation2d(4.06, 4.0); // Example hub position, replace with actual coordinates


  public AutoAimTurretCommand(TurretSubsystem turret, SwerveSubsystem swerve, Optional<Alliance> alliance) {
    this.turret = turret;
    this.swerve = swerve;
    this.alliance = alliance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        hubPose = new Translation2d(4.06, 4.0);
      } else if (alliance.get() == Alliance.Red) {
        hubPose = new Translation2d(16 - 4.06, 4.0); //random numbers for hubs
      }
  }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putString("Turret Hub Pose", hubPose.toString());
    turret.setAngle(
        TurretCalc.calculateTurretAngle(swerve.getStateCopy().Pose, hubPose)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
