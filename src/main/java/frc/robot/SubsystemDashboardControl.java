// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.ShooterHoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import gg.questnav.questnav.protos.generated.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SubsystemDashboardControl extends Command {

  ShooterSubsystem shooterSubsystem;
  ShooterHoodSubsystem shooterHoodSubsystem;
  /** Creates a new SubsystemDashboardControl. */
  public SubsystemDashboardControl(ShooterSubsystem _shooterSubsystem, ShooterHoodSubsystem _shooterHoodSubsystem) {
    this.shooterSubsystem = _shooterSubsystem;
    this.shooterHoodSubsystem = _shooterHoodSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterHoodSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Setpoint", 30);
    SmartDashboard.putNumber("frc3620/Shooter/Shooter RPM Setpoint", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        double hoodAngle = SmartDashboard.getNumber("frc3620/ShooterHood/Hood Angle Setpoint", 0);
        double shooterRPM = SmartDashboard.getNumber("frc3620/Shooter/Shooter RPM Setpoint", 0);

        shooterHoodSubsystem.setAngleDirect(Degrees.of(hoodAngle));
        shooterSubsystem.setVelocityDirect(RPM.of(shooterRPM));

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
