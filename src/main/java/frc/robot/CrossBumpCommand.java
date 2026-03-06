// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrossBumpCommand extends Command {

  private final SwerveSubsystem swerve;

  private final double vx;
  private final double vy;
  private final double vRot;

  SwerveRequest.FieldCentric executeRequest;
  SwerveRequest.FieldCentric endRequest;
  private final Timer timer = new Timer();

  /** Creates a new DriveToAdvancedCommand. */
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

    // creates a new drive request for execute
    executeRequest = new SwerveRequest.FieldCentric()
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withRotationalRate(vRot);

    // creates new drive request for end
    endRequest = new SwerveRequest.FieldCentric()
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerve.applyRequest(() -> executeRequest);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerve.applyRequest(() -> endRequest);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (timer.hasElapsed(.5) && Math.abs(swerve.getPigeon2().getPitch().getValueAsDouble()) < 5);
  }
}
