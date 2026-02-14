// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.QuestNavSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetQuestNavPoseFromMegaTag1Command extends Command {
  boolean resetQN;
  TaggedLogger logger = LoggingMaster.getLogger(getClass());


  /** Creates a new SetQuestNavPoseFromMegaTag1Command. */
  public SetQuestNavPoseFromMegaTag1Command() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetQN = false;
    logger.info("initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int mostTargets = 0; 
    Pose2d pose = null; 
    String used = null; 
    for (var cameraData : RobotContainer.limelightSubsystem.getAllCameraData()) {
      if (cameraData.shouldUseThisCamera()) {
        PoseEstimate pe = cameraData.megaTag1.getPoseEstimate();
        if (pe !=null && pe.tagCount > mostTargets){
          mostTargets = pe.tagCount;
          pose = pe.pose;
          used = cameraData.getLimelightName();
        }
      }
    }
    if (pose != null) {
      if (RobotContainer.questNavSubsystem != null){
        QuestNavSubsystem qn = RobotContainer.questNavSubsystem;
        qn.setQuestNavPose(pose);
        resetQN = true;

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.info("ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return resetQN;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
