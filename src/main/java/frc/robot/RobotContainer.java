package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.JoystickAnalogButton;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.XBoxConstants;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.StateTransition;
import frc.robot.fsm.states.IState;
import frc.robot.fsm.states.PassingState;
import frc.robot.fsm.states.ScoringState;
import frc.robot.Subsystems.QuestNavSubsystem;
// frc.robot.FSM.States;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Generated.TunerConstants;
import frc.robot.Helpers.FieldTriggers;
import frc.robot.Generated.ChudbotTunerConstants;
import frc.robot.Generated.JoeHannTunerConstants;
import frc.robot.Subsystems.SwerveSubsystem;

import frc.robot.Subsystems.IntakeRollerSubsytem;
import frc.robot.Subsystems.IntakeShoulderSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;

import frc.robot.Subsystems.ShooterTriggerSubsystem;
import frc.robot.Subsystems.ShooterHoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.PreshooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static TaggedLogger logger = LoggingMaster.getLogger(RobotContainer.class);

  // States
  private PassingState passingState;
  private ScoringState scoringState;

  private StateMachine stateMachine;
  private FieldTriggers fieldTriggers;

  private double MaxSpeed = ChudbotTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
  private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // maxangular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveTelemetry swerveLogger = new SwerveTelemetry(MaxSpeed);

  public static SwerveSubsystem swerveSubsystem;

  public static LimelightSubsystem limelightSubsystem;

  public static QuestNavSubsystem questNavSubsystem;

  private SendableChooser<Command> autoChooser;

  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  Alert missingDevicesAlert = new Alert("Diagnostics", "", Alert.AlertType.kWarning);

  // subsystems here

  // joysticks here....
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;
  public static Joystick operatorKeyboard;

  public TurretSubsystem turretSubsystem;
  public ClimberSubsystem climberSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public IntakeShoulderSubsystem intakeShoulderSubsystem;
  public IntakeRollerSubsytem intakeRollerSubsystem;
  public SpindexerSubsystem spindexerSubsystem;
  public ShooterHoodSubsystem shooterHoodSubsystem;
  public static ShooterTriggerSubsystem shooterTriggerSubsystem;
  public PreshooterSubsystem preshooterSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();
    logger.info("CAN Devices = {}", canDeviceFinder.getDeviceSet());

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());
    Utilities.logMetadataToDataLog("Robot", robotParameters.getName());

    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    
    makeJoysticks();
    
    makeSubsystems();
    if (!canDeviceFinder.getMissingDeviceSet().isEmpty()) {
      missingDevicesAlert.set(true);
      missingDevicesAlert.setText("Missing from CAN bus: " + canDeviceFinder.getMissingDeviceSet());
    }
    
    makeStates();
    makeStateTransitions();
    makeStateMachine();
    

    setupSmartDashboardCommands();

    setupPathPlannerCommands();
    setUpAutonomousCommands();

    configureButtonBindings();

    FollowPathCommand.warmupCommand().schedule();

    // default commands
    turretSubsystem.setDefaultCommand(turretSubsystem.setAngle(Degrees.of(0)));
    //climberSubsystem.setDefaultCommand(climberSubsystem.set(0));

    //shooterSubsystem.setDefaultCommand(shooterSubsystem.setVelocity(RPM.of(0)));
    //intakeShoulderSubsystem.setDefaultCommand(intakeShoulderSubsystem.setAngle(Degrees.of(90)));
    intakeRollerSubsystem.setDefaultCommand(intakeRollerSubsystem.rollersOff());
    spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setVelocityCommand(RPM.of(0)));
    shooterHoodSubsystem.setDefaultCommand(shooterHoodSubsystem.setAngle(Degrees.of(45)));
    shooterTriggerSubsystem.setDefaultCommand(shooterTriggerSubsystem.setSpeed(0.0));
    //preshooterSubsystem.setDefaultCommand(preshooterSubsystem.setVelocityCommand(RPM.of(0)));
  }

  private void makeSubsystems() {
    swerveSubsystem = configureSwerveDrive();
    questNavSubsystem = new QuestNavSubsystem(swerveSubsystem, new Pose3d());
    limelightSubsystem = new LimelightSubsystem();

    turretSubsystem = new TurretSubsystem();
    //climberSubsystem = new ClimberSubsystem();
    //shooterSubsystem = new ShooterSubsystem();
    //intakeShoulderSubsystem = new IntakeShoulderSubsystem();
    intakeRollerSubsystem = new IntakeRollerSubsytem();
    spindexerSubsystem = new SpindexerSubsystem();
    shooterHoodSubsystem = new ShooterHoodSubsystem();
    shooterTriggerSubsystem = new ShooterTriggerSubsystem();
    //preshooterSubsystem = new PreshooterSubsystem();
  }

  private SwerveSubsystem configureSwerveDrive() {
    String serialNumber = RobotController.getSerialNumber();

    if (serialNumber.equals("0326F30D") || serialNumber.equals("0327BA54")) {
      SmartDashboard.putString("Robot Serial", serialNumber);
      SmartDashboard.putString("Robot Variant", "Chudbot");
      return ChudbotTunerConstants.createDrivetrain();
    } else if (serialNumber.equals("03063D7E")) {
      SmartDashboard.putString("Robot Serial", serialNumber);
      SmartDashboard.putString("Robot Variant", "JoeHann");
      return JoeHannTunerConstants.createDrivetrain();
    } else {
      SmartDashboard.putString("Robot Serial", serialNumber);
      SmartDashboard.putString("Robot Variant", "Other");
      return TunerConstants.createDrivetrain();
    }

  }

  private void makeStates() {
    passingState = new PassingState();
    scoringState = new ScoringState();
  }

  private void makeStateTransitions() {
    fieldTriggers = new FieldTriggers(swerveSubsystem);

    passingState.addTransition(new StateTransition(
        fieldTriggers.enterOpponentAlliance,
        scoringState));

    scoringState.addTransition(new StateTransition(
        fieldTriggers.enterOurAlliance,
        passingState));
  }

  private void makeStateMachine() {
    stateMachine = new StateMachine(passingState);
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }

  private void makeJoysticks() {
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);
    operatorKeyboard = new Joystick(2);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    swerveSubsystem.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerveSubsystem.applyRequest(
            () -> drive.withVelocityX(MathUtil.applyDeadband(-driverJoystick.getRawAxis(1), 0.1) * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y
                                                                                                             // (forward)
                .withVelocityY(MathUtil.applyDeadband(-driverJoystick.getRawAxis(0), 0.1) * MaxSpeed) // Drive left with
                                                                                                      // negative X
                                                                                                      // (left)
                .withRotationalRate(-driverJoystick.getRawAxis(4) * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)

        ));

    // fix questnav correction command
    CommandScheduler.getInstance().schedule(new SetQuestNavPoseFromMegaTag1Command());
    CommandScheduler.getInstance().schedule(
        new InstantCommand(() -> swerveSubsystem.getPigeon2().setYaw(limelightSubsystem.getMegaTag1Rotation().getDegrees()))
            .andThen(new InstantCommand(() -> swerveSubsystem.seedFieldCentric(limelightSubsystem.getMegaTag1Rotation()))));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true));

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_A)
        .whileTrue(swerveSubsystem.applyRequest(() -> brake));
    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_B)
        .whileTrue(swerveSubsystem.applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverJoystick.getRawAxis(1), -driverJoystick.getRawAxis(0)))));

    swerveSubsystem.registerTelemetry(swerveLogger::telemeterize);

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER)
        .whileTrue(swerveSubsystem.applyRequest(() -> drive.withVelocityX(0.2) // Drive forward with negative Y
                                                                               // (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive coun
        ));

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_A)
        .whileTrue(turretSubsystem.setAngle(Degrees.of(45)));

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_B)
        .whileTrue(turretSubsystem.setAngle(Degrees.of(-45)));

    //new JoystickButton(driverJoystick, XBoxConstants.BUTTON_X).whileTrue(climberSubsystem.setHeight(Inches.of(48)));
    //new JoystickButton(driverJoystick, XBoxConstants.BUTTON_Y).whileTrue(climberSubsystem.setHeight(Inches.of(0)));

    new JoystickAnalogButton(driverJoystick, XBoxConstants.AXIS_LEFT_TRIGGER);
        //.onTrue(shooterSubsystem.setVelocity(RPM.of(600)));
    new JoystickButton(driverJoystick, 3);
        //.whileTrue(intakeShoulderSubsystem.setAngle(Degrees.of(0)));
    new JoystickButton(driverJoystick, 4).whileTrue(shooterTriggerSubsystem.setSpeed(1500.0));



  }

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData(new xxxxCommand());
  }

  public void setUpAutonomousCommands() {
    if (swerveSubsystem != null) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = null;
    }

    if (autoChooser != null) {
      SmartDashboard.putData("Auto Mode", autoChooser);
    }
  }

  public Command getAutonomousCommand() {
    if (autoChooser != null) {
      return autoChooser.getSelected();
    }
    return null;
  }

  public static void setupPathPlannerCommands() {
    NamedCommands.registerCommand("Reset QuestNav", new SetQuestNavPoseFromMegaTag1Command());
  }

  /**
   * Determine if this robot is a competition robot.
   * <p>
   * <li>
   * <ul>
   * It is if it's connected to an FMS.
   * </ul>
   * <ul>
   * It is if it is missing a grounding jumper on DigitalInput 0.
   * </ul>
   * <ul>
   * It is if the robot_parameters.json says so for this MAC address.
   * </ul>
   * </li>
   * </p>
   *
   * @return true if this robot is a competition robot.
   */
  @SuppressWarnings({ "unused", "RedundantIfStatement", "PointlessBooleanExpression" })
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (robotParameters.isCompetitionRobot()) {
      return true;
    }

    return false;
  }

  /**
   * Determine if we should make software objects, even if the device does
   * not appear on the CAN bus.
   * <p>
   * <li>
   * <ul>
   * We should if it's connected to an FMS.
   * </ul>
   * <ul>
   * We should if it is missing a grounding jumper on DigitalInput 0.
   * </ul>
   * <ul>
   * We should if the robot_parameters.json says so for this MAC address.
   * </ul>
   * </li>
   * </p>
   *
   * @return true if we should make all software objects for CAN devices
   */
  @SuppressWarnings({ "unused", "RedundantIfStatement" })
  public static boolean shouldMakeAllCANDevices() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

}