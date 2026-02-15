package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.odo.OdoIdsFlySky;
import org.usfirst.frc3620.odo.OdoIdsXBox;
import org.usfirst.frc3620.odo.OdoJoystick;
import org.usfirst.frc3620.odo.OdoJoystick.JoystickType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;

import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.RobotModeChangeListener;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;

import java.util.EnumSet;
import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.BlinkyLightsSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.HealthSubsystem;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.StateTransition;
import frc.robot.fsm.states.ClimbingState;
import frc.robot.fsm.states.DeadeyeState;
import frc.robot.fsm.states.HoardingState;
import frc.robot.fsm.states.PassingState;
import frc.robot.fsm.states.ScoringState;
import frc.robot.Subsystems.QuestNavSubsystem;
// frc.robot.FSM.States;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Generated.TunerConstants;
import frc.robot.Helpers.ButtonTriggers;
import frc.robot.Helpers.FMSTriggers;
import frc.robot.Helpers.FieldTriggers;
import frc.robot.Generated.ChudbotTunerConstants;
import frc.robot.Generated.RaptorTunerConstants;
import frc.robot.Subsystems.SwerveSubsystem;

import frc.robot.Subsystems.IntakeRollerSubsytem;
import frc.robot.Subsystems.IntakeShoulderSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

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
public class RobotContainer implements RobotModeChangeListener {
  public final static TaggedLogger logger = LoggingMaster.getLogger(RobotContainer.class);

  // States
  private PassingState passingState;
  private ScoringState scoringState;
  private ClimbingState climbingState;
  private DeadeyeState deadeyeState;
  private HoardingState hoardingState;

  private static StateMachine stateMachine;
  private FieldTriggers fieldTriggers;
  private FMSTriggers fmsTriggers;
  private ButtonTriggers buttonTriggers;

  private Optional<Alliance> alliance = DriverStation.getAlliance();

  private double MaxSpeed = ChudbotTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
  private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // maxangular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private SwerveRequest.FieldCentric drive;
  private SwerveRequest.SwerveDriveBrake brake;
  private SwerveRequest.PointWheelsAt point;
  private SwerveTelemetry swerveLogger;

  public static HealthSubsystem healthSubsystem;

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
  public static OdoJoystick driverJoystick;
  public static OdoJoystick operatorJoystick;
  public static Joystick operatorKeyboard;

  public TurretSubsystem turretSubsystem;
  public ClimberSubsystem climberSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public static IntakeShoulderSubsystem intakeShoulderSubsystem;
  public IntakeRollerSubsytem intakeRollerSubsystem;

  public ShooterHoodSubsystem shooterHoodSubsystem;
  public PreshooterSubsystem preshooterSubsystem;
  public BlinkyLightsSubsystem blinkyLightsSubsystem;

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
    // climberSubsystem.setDefaultCommand(climberSubsystem.set(0));

    shooterSubsystem.setDefaultCommand(shooterSubsystem.setVelocity(RPM.of(0)));
    intakeShoulderSubsystem.setDefaultCommand(intakeShoulderSubsystem.setAngle(Degrees.of(90)));
    intakeRollerSubsystem.setDefaultCommand(intakeRollerSubsystem.rollersOff());

    shooterHoodSubsystem.setDefaultCommand(shooterHoodSubsystem.setAngle(Degrees.of(30)));
    // preshooterSubsystem.setDefaultCommand(preshooterSubsystem.setVelocityCommand(RPM.of(0)));

    Robot.addRobotModeChangeListener(this);
  }

  private void makeSubsystems() {
    healthSubsystem = new HealthSubsystem();

    swerveSubsystem = configureSwerveDrive();
    if (swerveSubsystem != null) {
      /* Setting up bindings for necessary control of the swerve drive platform */
      drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
      brake = new SwerveRequest.SwerveDriveBrake();
      point = new SwerveRequest.PointWheelsAt();

      swerveLogger = new SwerveTelemetry(MaxSpeed);

      sendSwerveSubsystemToHealthSubsystem();
    }

    questNavSubsystem = new QuestNavSubsystem(swerveSubsystem, new Pose3d());
    limelightSubsystem = new LimelightSubsystem();

    turretSubsystem = new TurretSubsystem();
    climberSubsystem = new ClimberSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    intakeShoulderSubsystem = new IntakeShoulderSubsystem();
    intakeRollerSubsystem = new IntakeRollerSubsytem();

    shooterHoodSubsystem = new ShooterHoodSubsystem();
    preshooterSubsystem = new PreshooterSubsystem();
    blinkyLightsSubsystem = new BlinkyLightsSubsystem();

    healthSubsystem.dumpDatabase();
  }

  private SwerveSubsystem configureSwerveDrive() {
    String serialNumber = RobotController.getSerialNumber();
    SmartDashboard.putString("frc3620/Robot Serial", serialNumber);
    String robotVariant = robotParameters.getVariant();
    SmartDashboard.putString("frc3620/Robot Variant", robotVariant);

    Class<?> tunerConstantsClass;
    if (robotVariant.toLowerCase().equals("chudbot")) {
      tunerConstantsClass = ChudbotTunerConstants.class;
    } else if (robotVariant.toLowerCase().equals("raptor")) {
      tunerConstantsClass = RaptorTunerConstants.class;
    } else {
      tunerConstantsClass = TunerConstants.class;
    }

    Integer motorId = Utilities.extractPrivateField(Integer.class, tunerConstantsClass, null, "kFrontLeftDriveMotorId");
    boolean shouldMakeSwerve = canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId,
        "Swerve Subsystem") || RobotContainer.shouldMakeAllCANDevices();
    SwerveSubsystem rv = null;
    if (shouldMakeSwerve) {
      rv = Utilities.callMethod(SwerveSubsystem.class, tunerConstantsClass, null, "createDrivetrain");
    }
    logger.info("looked for swerve motor {}, got {}, made swerve {} from {}", motorId, shouldMakeSwerve, rv.getClass().getName(), tunerConstantsClass);
    return rv;
  }

  private void makeStates() {
    passingState = new PassingState();
    scoringState = new ScoringState();
    climbingState = new ClimbingState();
    deadeyeState = new DeadeyeState();
    hoardingState = new HoardingState();

  }

  private void makeStateTransitions() {
    if (swerveSubsystem == null) {
      return;
    }
    fieldTriggers = new FieldTriggers(() -> swerveSubsystem.getState().Pose);
    fmsTriggers = new FMSTriggers(alliance);
    buttonTriggers = new ButtonTriggers(driverJoystick);

    passingState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterOurAllianceZone),
        scoringState));

    passingState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterDeadZone),
        hoardingState));
    passingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterDeadZone),
        hoardingState));
    passingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterOurAllianceZone),
        hoardingState));

    scoringState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterNeutralOutpost),
        passingState));
    scoringState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterNeutralDepot),
        passingState));

    scoringState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterNeutralDepot),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterNeutralOutpost),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterDeadZone),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterOurAllianceZone),
        hoardingState));

    /*
     * scoringState.addTransition(new StateTransition(
     * fmsTriggers.isEndgame.and(fieldTriggers.enterClimbZone).and(buttonTriggers.
     * climb),
     * climbingState));
     */
    hoardingState.addTransition(new StateTransition(
        fmsTriggers.isActivePeriod.and(fieldTriggers.enterOurAllianceZone),
        scoringState));

    hoardingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterOpponentDepot),
        passingState));
    hoardingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterOpponentOutpost),
        passingState));
    hoardingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterNeutralDepot),
        passingState));
    hoardingState.addTransition(new StateTransition(
        fmsTriggers.isInactivePeriod.and(fieldTriggers.enterNeutralOutpost),
        passingState));
  }

  private void makeStateMachine() {
    stateMachine = new StateMachine(passingState);
  }

  public static StateMachine getStateMachine() {
    return stateMachine;
  }

  private void makeJoysticks() {
    driverJoystick = new OdoJoystick(new Joystick(0));
    operatorJoystick = new OdoJoystick(new Joystick(1));
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
    if (swerveSubsystem != null) {
      swerveSubsystem.setDefaultCommand(
          // Drivetrain will execute this command periodically
          swerveSubsystem.applyRequest(
              () -> drive
                  // Drive forward with negative Y (forward)
                  .withVelocityX(MathUtil.applyDeadband(
                      -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_Y, OdoIdsXBox.AxisId.LEFT_Y), 0.1) * MaxSpeed)
                  // Drive with negative X (left)
                  .withVelocityY(MathUtil.applyDeadband(
                      -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_X, OdoIdsXBox.AxisId.LEFT_X), 0.1) * MaxSpeed) // Drive
                  // Drive counterclockwise with negative X (left) left
                  .withRotationalRate(-driverJoystick.getAxis(OdoIdsFlySky.AxisId.RIGHT_X, OdoIdsXBox.AxisId.RIGHT_X)
                      * MaxAngularRate))
              .withName("Drive from Joysticks"));

      // Idle while the robot is disabled. This ensures the configured
      // neutral mode is applied to the drive motors while disabled.
      final var idle = new SwerveRequest.Idle();
      RobotModeTriggers.disabled().whileTrue(
          swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true).withName("Idle"));

      driverJoystick.button(() -> false, OdoIdsXBox.ButtonId.A)
          .whileTrue(swerveSubsystem.applyRequest(() -> brake).withName("Breaks"));
          driverJoystick.button(() -> false, OdoIdsXBox.ButtonId.B)
          .whileTrue(swerveSubsystem.applyRequest(() -> point.withModuleDirection(new Rotation2d( //
          -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_Y, OdoIdsXBox.AxisId.LEFT_Y), //
          -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_Y, OdoIdsXBox.AxisId.LEFT_Y) //
          ))).withName("Point"));
          
          driverJoystick.button(() -> false, OdoIdsXBox.ButtonId.X)
              .whileTrue(intakeShoulderSubsystem.setAngle(Degrees.of(0)).withName("Intake shoulder test"));
      swerveSubsystem.registerTelemetry(swerveLogger::telemeterize);
      
      driverJoystick.button(() -> false, OdoIdsXBox.ButtonId.RIGHT_BUMPER)
          .whileTrue(swerveSubsystem.applyRequest(() -> drive
              // Drive forward with negative Y(forward)
              .withVelocityX(0.2)
              // Drive left with negative X (left)
              .withVelocityY(0)
              .withRotationalRate(0) // Drive coun
          ).withName("Drive Slow"));

          
      CommandScheduler.getInstance().schedule(
          new SetPigeonFromMegaTag1Command().withName("Reset Pigeon from MegaTag1").ignoringDisable(true));
    }

    // fix questnav correction command
    CommandScheduler.getInstance().schedule(new SetQuestNavPoseFromMegaTag1Command());

    operatorJoystick.button(OdoIdsXBox.ButtonId.A)
        .whileTrue(shooterHoodSubsystem.setAngle(Degrees.of(45)));

    operatorJoystick.button(OdoIdsXBox.ButtonId.B)
        .whileTrue(shooterHoodSubsystem.setAngle(Degrees.of(35)));

    operatorJoystick.button(OdoIdsXBox.ButtonId.LEFT_BUMPER)
        .whileTrue(intakeRollerSubsystem.rollersOn());
    intakeRollerSubsystem.setDefaultCommand(intakeRollerSubsystem.rollersOff());

    operatorJoystick.button(OdoIdsXBox.ButtonId.X)
        .onTrue(new SetPigeonFromMegaTag1Command().withName("Reset Pigeon from MegaTag1").ignoringDisable(true)
            .andThen(new SetQuestNavPoseFromMegaTag1Command().withName("Reset QuestNav from MegaTag1"))
            .ignoringDisable(true));
  }

  public void processRobotModeChange(RobotMode currentRobotMode, RobotMode previousRobotMode) {
    if (currentRobotMode == RobotMode.TELEOP) {
      Joystick realDriverJoystick = driverJoystick.getRealJoystick();
      String driveControllerName = realDriverJoystick.getName();
      int n_axes = realDriverJoystick.getAxisCount();
      int n_buttons = realDriverJoystick.getButtonCount();
      logger.info("Drive Controller '{}', {}connected, {} axes, {} buttons", driveControllerName,
          realDriverJoystick.isConnected() ? "" : "not ", n_axes, n_buttons);
      if (driveControllerName.startsWith("Flysky")) {
        driverJoystick.setJoystickType(JoystickType.A);
      } else {
        driverJoystick.setJoystickType(JoystickType.B);
      }
    }
  }

  private void setupSmartDashboardCommands() {
    SmartDashboard.putData("frc3620/ShooterHood/Calibrate", shooterHoodSubsystem.calibrate());
    SmartDashboard.putData("frc3620/ShooterHood/DashboardControl", shooterHoodSubsystem.setAngleDashboardCommand());
    SmartDashboard.putData("frc3620/Shooter/DashboardControl", shooterSubsystem.setVelocityDashbaordCommand());
    // SmartDashboard.putData(new xxxxCommand());
    SmartDashboard.putData(
    "frc3620/IntakeShoulder/ Set 60",
    intakeShoulderSubsystem.setAngle(Degrees.of(60))
);

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

  void sendSwerveSubsystemToHealthSubsystem() {
    var modules = swerveSubsystem.getModules();
    var locations = swerveSubsystem.getModuleLocations();
    for (var i = 0; i < modules.length; i++) {
      var module = modules[i];
      var location = locations[i];
      // X and Y seem swapped, but look at TunerConstants.
      // +X is to front of robot, +Y is to left
      /*
       * ·· +Y
       * -X [> +X (front)
       * ·· -Y
       */
      var location_name = ((location.getX() > 0) ? "Front" : "Back") + ((location.getY() > 0) ? "Left" : "Right");

      CoreTalonFX steer_motor = module.getSteerMotor();
      healthSubsystem.addMotorToWatch(steer_motor, "Swerve/" + location_name + "/steer",
          HealthSubsystem.healthOptionsForCTRESwerveMotors);
      CoreTalonFX drive_motor = module.getDriveMotor();
      healthSubsystem.addMotorToWatch(drive_motor, "Swerve/" + location_name + "/drive",
          HealthSubsystem.healthOptionsForCTRESwerveMotors);
      CANcoder cancoder = module.getEncoder();
      healthSubsystem.addCTRESensorToWatch(cancoder, "Swerve/" + location_name + "/cancoder",
          HealthSubsystem.healthOptionsForCTRESwerveSensors);
    }
    Pigeon2 pigeon = swerveSubsystem.getPigeon2();
    healthSubsystem.addCTRESensorToWatch(pigeon, "Swerve/pigeon", HealthSubsystem.healthOptionsForCTRESwerveSensors);
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
