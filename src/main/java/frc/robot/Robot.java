package frc.robot;

import org.tinylog.TaggedLogger;

import org.usfirst.frc3620.*;
import org.usfirst.frc3620.logger.LoggingMaster;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private TaggedLogger logger;

  static private RobotMode currentRobotMode = RobotMode.INIT, previousRobotMode;

  public Robot() {
    // get data logging going
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(false));
    DataLogManager.start();

    logger = LoggingMaster.getLogger(getClass());
    logger.info ("I'm alive! {}", GitNess.gitDescription());
    Utilities.logMetadataToDataLog();

    Utilities.addDataLogForNT("frc3620");
    Utilities.addDataLogForNT("SmartDashboard/frc3620");

    // whenever a command initializes, the function declared below will run.
    CommandScheduler.getInstance().onCommandInitialize(command ->
            logger.info("Initialized {}", command.getName()));

    // whenever a command ends, the function declared below will run.
    CommandScheduler.getInstance().onCommandFinish(command ->
            logger.info("Ended {}", command.getName()));

    // whenever a command ends, the function declared below will run.
    CommandScheduler.getInstance().onCommandInterrupt(command ->
            logger.info("Interrupted {}", command.getName()));
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    FileSaver.add("networktables.json");

    enableLiveWindowInTest(true);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Runtime rt = Runtime.getRuntime();
    SmartDashboard.putNumber("frc3620/heap/free", rt.freeMemory());
    SmartDashboard.putNumber("frc3620/heap/total", rt.totalMemory());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    processRobotModeChange(RobotMode.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    logCANBusIfNecessary(); // don't do this when enabled; unnecessary overhead
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    logCANBusIfNecessary();

    processRobotModeChange(RobotMode.AUTONOMOUS);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    logCANBusIfNecessary();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    processRobotModeChange(RobotMode.TELEOP);
    logMatchInfo();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    logCANBusIfNecessary();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    processRobotModeChange(RobotMode.TEST);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /*
  * this routine gets called whenever we change modes
  */
  void processRobotModeChange(RobotMode newMode) {
    logger.info("Switching from {} to {}", currentRobotMode, newMode);
    
    previousRobotMode = currentRobotMode;
    currentRobotMode = newMode;

    SmartDashboard.putString("frc3620/mode", newMode.toString());
    SmartDashboard.putNumber("frc3620/modeInt", newMode.ordinal());

    // if any subsystems need to know about mode changes, let
    // them know here.
    // exampleSubsystem.processRobotModeChange(newMode);
    
  }

  public static RobotMode getCurrentRobotMode(){
    return currentRobotMode;
  }

  public static RobotMode getPreviousRobotMode(){
    return previousRobotMode;
  }

  void logMatchInfo() {
    if (DriverStation.isFMSAttached()) {
      logger.info("FMS attached. Event name {}, match type {}, match number {}, replay number {}", 
        DriverStation.getEventName(),
        DriverStation.getMatchType(),
        DriverStation.getMatchNumber(),
        DriverStation.getReplayNumber());
    }
    logger.info("Alliance {}, position {}", DriverStation.getAlliance(), DriverStation.getLocation());
  }

  private boolean hasCANBusBeenLogged;
  
  void logCANBusIfNecessary() {
    if (!hasCANBusBeenLogged) {
      if (DriverStation.isDSAttached()) {
        logger.info("CAN bus: {}", RobotContainer.canDeviceFinder.getDeviceSet());
        var missingDevices = RobotContainer.canDeviceFinder.getMissingDeviceSet();
        if (!missingDevices.isEmpty()) {
          logger.warn("Missing devices: {}", missingDevices);
        }
        hasCANBusBeenLogged = true;
      }
    }
  }

}