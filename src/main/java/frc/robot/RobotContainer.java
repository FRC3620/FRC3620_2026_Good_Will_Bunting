package frc.robot;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

import dev.doglog.DogLog;

import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.JoystickAnalogButton;
import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.RobotModeChangeListener;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.BlinkyLightsSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.ConveyerSubsystem;
import frc.robot.Subsystems.HealthSubsystem;
import frc.robot.Subsystems.IntakeAgitatorSubsytem;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Generated.TunerConstants;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Helpers.ButtonTriggers;
import frc.robot.Helpers.FMSTriggers;
import frc.robot.Helpers.FieldTriggers;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
import frc.robot.Generated.ChudbotTunerConstants;
import frc.robot.Generated.RaptorTunerConstants;
import frc.robot.Subsystems.SwerveSubsystem;

import frc.robot.Subsystems.IntakeRollerSubsytem;
import frc.robot.Subsystems.IntakeShoulderSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

import frc.robot.Subsystems.ShooterHoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.IntakeShoulderSubsystem.IntakeShoulderPositions;
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

  private Optional<Alliance> alliance;

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
  public ConveyerSubsystem conveyerSubsystem;
  public IntakeAgitatorSubsytem intakeAgitatorSubsystem;

  public ShooterHoodSubsystem shooterHoodSubsystem;
  public PreshooterSubsystem preshooterSubsystem;
  public BlinkyLightsSubsystem blinkyLightsSubsystem;

  // hardware here
  public static PowerDistribution powerDistribution;

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

    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.REV_PDH, 1, "PDH")
        || RobotContainer.shouldMakeAllCANDevices()) {
      powerDistribution = new PowerDistribution(1, ModuleType.kRev);
      DogLog.setPdh(powerDistribution);
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
    // turretSubsystem.setDefaultCommand(turretSubsystem.setAngle(() ->
    // Degrees.of(0)));
    climberSubsystem.setDefaultCommand(climberSubsystem.set(0));

    // shooterSubsystem.setDefaultCommand(shooterSubsystem.setVelocity(() ->
    // RPM.of(0)));
    intakeShoulderSubsystem.setDefaultCommand(intakeShoulderSubsystem.idle());
    intakeRollerSubsystem.setDefaultCommand(intakeRollerSubsystem.rollersOff());
    conveyerSubsystem.setDefaultCommand(conveyerSubsystem.setSpeed(() -> RPM.of(0)));
    // shooterHoodSubsystem.setDefaultCommand(shooterHoodSubsystem.setAngle(() ->
    // Degrees.of(30)));
    preshooterSubsystem.setDefaultCommand(preshooterSubsystem.createSetVelocityCommand(() -> RPM.of(0)));
    intakeAgitatorSubsystem.setDefaultCommand(intakeAgitatorSubsystem.agitatorOff());

    Robot.addRobotModeChangeListener(this);
  }

  private void makeSubsystems() {
    healthSubsystem = new HealthSubsystem();

    swerveSubsystem = configureSwerveDrive();
    if (swerveSubsystem != null) {
      /* Setting up bindings for necessary control of the swerve drive platform */
      drive = new SwerveRequest.FieldCentric()
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
    intakeAgitatorSubsystem = new IntakeAgitatorSubsytem();

    shooterHoodSubsystem = new ShooterHoodSubsystem();
    preshooterSubsystem = new PreshooterSubsystem();
    conveyerSubsystem = new ConveyerSubsystem();
    blinkyLightsSubsystem = new BlinkyLightsSubsystem();

    // healthSubsystem.dumpDatabase();
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
    logger.info("looked for swerve motor {}, got {}", motorId, shouldMakeSwerve);
    SwerveSubsystem rv = null;
    if (shouldMakeSwerve) {
      logger.info("making swerve from {}", tunerConstantsClass);
      rv = Utilities.callMethod(SwerveSubsystem.class, tunerConstantsClass, null, "createDrivetrain");
    }
    if (rv == null) {
      logger.error("no swerve drive was created!");
    } else {
      logger.info("swerve drive = {}", rv);
    }
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
    SmartDashboard.putBoolean("frc3620/StateMachine/FMS triggers On", true);

    fieldTriggers = new FieldTriggers(() -> swerveSubsystem.getState().Pose);
    fmsTriggers = new FMSTriggers();
    buttonTriggers = new ButtonTriggers(driverJoystick);

    passingState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOurAllianceZone),
        scoringState));

    passingState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterDeadZone),
        hoardingState));
    passingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterDeadZone),
        hoardingState));
    passingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOurAllianceZone),
        hoardingState));

    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralOutpost),
        passingState));
    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralDepot),
        passingState));

    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralDepot),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralOutpost),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterDeadZone),
        hoardingState));
    scoringState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOurAllianceZone),
        hoardingState));

    /*
     * scoringState.addTransition(new StateTransition(
     * fmsTriggers.isEndgame.and(fieldTriggers.enterClimbZone).and(buttonTriggers.
     * climb),
     * climbingState));
     */
    hoardingState.addTransition(new StateTransition(
        (fmsTriggers.isActivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOurAllianceZone),
        scoringState));

    hoardingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOpponentDepot),
        passingState));
    hoardingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterOpponentOutpost),
        passingState));
    hoardingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralDepot),
        passingState));
    hoardingState.addTransition(new StateTransition(
        (fmsTriggers.isInactivePeriod.or(fmsTriggers.fmsTriggersOff)).and(fieldTriggers.enterNeutralOutpost),
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

  private void configureSwerveButtonBindings() {
    swerveSubsystem.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerveSubsystem.applyRequest(
            () -> drive
                // Drive forward with negative Y (forward)
                .withVelocityX(MathUtil.applyDeadband(
                    -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_Y, OdoIdsXBox.AxisId.LEFT_Y), 0.05) * MaxSpeed)
                // Drive with negative X (left)
                .withVelocityY(MathUtil.applyDeadband(
                    -driverJoystick.getAxis(OdoIdsFlySky.AxisId.LEFT_X, OdoIdsXBox.AxisId.LEFT_X), 0.05) * MaxSpeed) // Drive
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

    /*
     * SWERVE SYSID
     * operatorJoystick.button(OdoIdsXBox.ButtonId.X).and(operatorJoystick.button(
     * OdoIdsXBox.ButtonId.LEFT_BUMPER))
     * .whileTrue(
     * swerveSubsystem.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.
     * SysIdRoutine.Direction.kForward));
     * operatorJoystick.button(OdoIdsXBox.ButtonId.Y).and(operatorJoystick.button(
     * OdoIdsXBox.ButtonId.LEFT_BUMPER))
     * .whileTrue(
     * swerveSubsystem.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.
     * SysIdRoutine.Direction.kReverse));
     * operatorJoystick.button(OdoIdsXBox.ButtonId.A).and(operatorJoystick.button(
     * OdoIdsXBox.ButtonId.LEFT_BUMPER))
     * .whileTrue(
     * swerveSubsystem.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.
     * SysIdRoutine.Direction.kForward));
     * operatorJoystick.button(OdoIdsXBox.ButtonId.B).and(operatorJoystick.button(
     * OdoIdsXBox.ButtonId.LEFT_BUMPER))
     * .whileTrue(
     * swerveSubsystem.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.
     * SysIdRoutine.Direction.kReverse));
     */

    driverJoystick.button(OdoIdsFlySky.ButtonId.SWC, OdoIdsXBox.ButtonId.X)
        .onTrue(new SetPigeonFromMegaTag1Command().withName("Reset Pigeon from MegaTag1").ignoringDisable(true)
            .andThen(new SetQuestNavPoseFromMegaTag1Command().withName("Reset QuestNav from MegaTag1"))
            .ignoringDisable(true));
  }

  private void configureButtonBindings() {

    Trigger driverLeftTriggerFlySky = new Trigger(
      driverJoystick.button(OdoIdsFlySky.ButtonId.SWE, () -> false));
    Trigger driverRightTriggerFlySky = new Trigger(
      driverJoystick.button(OdoIdsFlySky.ButtonId.SWH, () -> false));

    Trigger driverIntakeSwitch = 
      driverJoystick.button(OdoIdsFlySky.ButtonId.SWA);

    Trigger rollersOnTrigger = new Trigger(
        () -> driverJoystick.getRawAxis(OdoIdsFlySky.AxisId.SWB) == -1.0);
    Trigger rollersOffTrigger = new Trigger(
        () -> driverJoystick.getRawAxis(OdoIdsFlySky.AxisId.SWB) == 0.0);
    Trigger rollersBackwardsTrigger = new Trigger(
        () -> driverJoystick.getRawAxis(OdoIdsFlySky.AxisId.SWB) == 1.0);

    if (swerveSubsystem != null) {
      /*
       **********************************************************************************
       **********************************************************************************
       **********************************************************************************
       * Put anything having to do with swerve in configureSwerveButtonBindings so we
       * can use mule boards!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       **********************************************************************************
       **********************************************************************************
       **********************************************************************************
       */
      configureSwerveButtonBindings();
    }

    /*
     **********************************************************************************
     **********************************************************************************
     **********************************************************************************
     * Check to make sure your subsystem is not null before adding commands that use
     * them!
     **********************************************************************************
     **********************************************************************************
     **********************************************************************************
     */

    if (shooterSubsystem != null) {
      /*
       * SHOOTER SYSID
       * operatorJoystick.button(OdoIdsXBox.ButtonId.X).and(operatorJoystick.button(
       * OdoIdsXBox.ButtonId.LEFT_BUMPER))
       * .whileTrue(
       * shooterSubsystem.sysIdDynamicForward());
       * operatorJoystick.button(OdoIdsXBox.ButtonId.Y).and(operatorJoystick.button(
       * OdoIdsXBox.ButtonId.LEFT_BUMPER))
       * .whileTrue(
       * shooterSubsystem.sysIdDynamicReverse());
       * operatorJoystick.button(OdoIdsXBox.ButtonId.A).and(operatorJoystick.button(
       * OdoIdsXBox.ButtonId.LEFT_BUMPER))
       * .whileTrue(
       * shooterSubsystem.sysIdQuasistaticForward());
       * operatorJoystick.button(OdoIdsXBox.ButtonId.B).and(operatorJoystick.button(
       * OdoIdsXBox.ButtonId.LEFT_BUMPER))
       * .whileTrue(
       * shooterSubsystem.sysIdQuasistaticReverse());
       */
    }

    if (intakeAgitatorSubsystem != null && conveyerSubsystem != null && preshooterSubsystem != null) {
      driverRightTriggerFlySky
          .whileTrue(
              preshooterSubsystem.createSetVelocityCommand(() -> RPM.of(2000))
                  .alongWith(conveyerSubsystem.setDutyCycle(0.8))
                  .alongWith(intakeAgitatorSubsystem.agitatorOn())
                  );
}

    if (intakeShoulderSubsystem != null) {
      driverLeftTriggerFlySky
        .whileTrue(
          intakeShoulderSubsystem.createJostleCommand()
          .alongWith(intakeRollerSubsystem.rollersOn())
        ).onFalse(
              intakeShoulderSubsystem.createSetPositionThenCoast(() -> IntakeShoulderPositions.OUT.getAngle())
        );

      driverIntakeSwitch
        .onTrue(
          intakeShoulderSubsystem.createSetPositionThenCoast(() -> IntakeShoulderPositions.OUT.getAngle()).withTimeout(3)
        )
        .onFalse(
          intakeShoulderSubsystem.createSetPositionCommand(() -> IntakeShoulderPositions.IN.getAngle())
        );
    }

    if (intakeRollerSubsystem != null) {
      rollersOnTrigger.onTrue(
        intakeRollerSubsystem.rollersOn()
      );
      rollersOffTrigger.onTrue(
        intakeRollerSubsystem.rollersOff()
      );
      rollersBackwardsTrigger.onTrue(
        intakeRollerSubsystem.rollersBackwards()
        .alongWith(
          intakeAgitatorSubsystem.agitatorBackwards()
        ).withName("Spit Balls Back")
      );
    }

  }

  public void processRobotModeChange(RobotMode currentRobotMode, RobotMode previousRobotMode) {
    alliance = DriverStation.getAlliance();
    if (currentRobotMode == RobotMode.TELEOP) {
      Joystick realDriverJoystick = driverJoystick.getRealJoystick();
      String driveControllerName = realDriverJoystick.getName();
      int n_axes = realDriverJoystick.getAxisCount();
      int n_buttons = realDriverJoystick.getButtonCount();
      logger.info("Drive Controller '{}', {}connected, {} axes, {} buttons", driveControllerName,
          realDriverJoystick.isConnected() ? "" : "not ", n_axes, n_buttons);
      if (driveControllerName.startsWith("FlySky")) {
        driverJoystick.setJoystickType(JoystickType.A);
      } else {
        driverJoystick.setJoystickType(JoystickType.B);
      }
    }
  }

  private void setupSmartDashboardCommands() {
    /*
     **********************************************************************************
     **********************************************************************************
     **********************************************************************************
     * Check to make sure your subsystem is not null before adding commands that use
     * them!
     **********************************************************************************
     **********************************************************************************
     **********************************************************************************
     */
    if (shooterHoodSubsystem != null) {
      SmartDashboard.putData("frc3620/ShooterHood/Calibrate", shooterHoodSubsystem.calibrate());
      SmartDashboard.putData("frc3620/ShooterHood/DashboardControl", shooterHoodSubsystem.setAngleDashboardCommand());

      if (swerveSubsystem != null) {
        SmartDashboard.putData("frc3620/ShooterHood/AutoAim", shooterHoodSubsystem.createAutoAngleToTargetCommand(
            new Translation3d(
                Feet.of(15.17),
                Feet.of(13.235),
                Feet.of(6)),
            () -> AllianceFlipUtil.apply(swerveSubsystem.getState().Pose),
            () -> AllianceFlipUtil.apply(ShotCalculator.calculateRobotVelocity(
                swerveSubsystem.getKinematics(),
                swerveSubsystem.getState(),
                swerveSubsystem.getPigeon2().getRotation2d()))));
      }
    }

    if (intakeRollerSubsystem != null) {
      SmartDashboard.putData("frc3620/IntakeRollers/rollersOff", intakeRollerSubsystem.rollersOff());
      SmartDashboard.putData("frc3620/IntakeRollers/rollersOn", intakeRollerSubsystem.rollersOn());
    }

    if (shooterSubsystem != null) {
      SmartDashboard.putData("frc3620/Shooter/DashboardControl",
          shooterSubsystem.setVelocityDashboardCommand().ignoringDisable(true));

      if (swerveSubsystem != null) {
        SmartDashboard.putData("frc3620/Shooter/AutoAim", shooterSubsystem.createSetSpeedToTargetCommand(
            new Translation3d(
                Feet.of(15.17),
                Feet.of(13.235),
                Feet.of(6)),
            () -> AllianceFlipUtil.apply(swerveSubsystem.getState().Pose),
            () -> AllianceFlipUtil.apply(ShotCalculator.calculateRobotVelocity(
                swerveSubsystem.getKinematics(),
                swerveSubsystem.getState(),
                swerveSubsystem.getPigeon2().getRotation2d()))));
      }
    }

    if (intakeShoulderSubsystem != null) {
      SmartDashboard.putData("frc3620/IntakeShoulder/DashboardControl",
          intakeShoulderSubsystem.setPositionDashboardCommand().ignoringDisable(true));
      SmartDashboard.putData("frc3620/IntakeShoulder/JostleCommand", intakeShoulderSubsystem.createJostleCommand());
    }

    if (conveyerSubsystem != null) {
      SmartDashboard.putData("frc3620/Conveyer/DashboardControl",
          conveyerSubsystem.setSpeedDashboardCommand().ignoringDisable(true));
    }

    if (preshooterSubsystem != null) {
      SmartDashboard.putData("frc3620/Preshooter/DashboardControl",
          preshooterSubsystem.setVelocityDashboardCommand().ignoringDisable(true));
    }

    if (turretSubsystem != null) {
      SmartDashboard.putData("frc3620/Turret/DashboardControl",
          turretSubsystem.setAngleDashboardCommand().ignoringDisable(true));
      if (swerveSubsystem != null) {
        SmartDashboard.putData("frc3620/Turret/AutoAim", turretSubsystem.createSetAngleToTargetCommand(
            new Translation2d(
                Feet.of(15.17),
                Feet.of(13.235)),
            () -> AllianceFlipUtil.apply(swerveSubsystem.getState().Pose),
            () -> AllianceFlipUtil.apply(ShotCalculator.calculateRobotVelocity(
                swerveSubsystem.getKinematics(),
                swerveSubsystem.getState(),
                swerveSubsystem.getPigeon2().getRotation2d()))));
      }
    }

    if (intakeAgitatorSubsystem != null) {
      SmartDashboard.putData("frc3620/intakeAgitator/AgitatorOn", intakeAgitatorSubsystem.agitatorOn());
      SmartDashboard.putData("frc3620/intakeAgitator/AgitatorOff", intakeAgitatorSubsystem.agitatorOff());
      SmartDashboard.putData("frc3620/intakeAgitator/AgitatorRev", intakeAgitatorSubsystem.agitatorBackwards());

    }
    SmartDashboard.putNumber("frc3620/ShotCalculator/TestInputs/RobotPoseXFt", 0);
    SmartDashboard.putNumber("frc3620/ShotCalculator/TestInputs/RobotPoseYFt", 0);
    SmartDashboard.putNumber("frc3620/ShotCalculator/TestInputs/RobotPoseRotationDegrees", 0);

    SmartDashboard.putNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityXFtps", 0);
    SmartDashboard.putNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityYFtps", 0);

    SmartDashboard.putData("frc3620/ShotCalculator/CalculateTestShot", new Command() {
      @Override
      public void initialize() {
        SmartDashboard.putNumber("frc3620/ShotCalculator/CalculatedShot/HoodAngleDegrees",
            ShotCalculator.calculateHoodAngle(
                new Translation3d(
                    Feet.of(15.17),
                    Feet.of(13.235),
                    Feet.of(6.0)),
                () -> new Pose2d(
                    Meters.convertFrom(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseXFt", 0),
                        Feet),
                    Meters.convertFrom(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseYFt", 0),
                        Feet),
                    Rotation2d.fromDegrees(
                        SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseRotationDegrees", 0))),
                () -> new VelocityVector(
                    FeetPerSecond
                        .of(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityXFtps", 0)),
                    FeetPerSecond
                        .of(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityYFtps", 0))))
                .in(Degrees));

        SmartDashboard.putNumber("frc3620/ShotCalculator/CalculatedShot/FlywheelVelocityRPM",
            ShotCalculator.calculateShooterSpeed(
                new Translation3d(
                    Feet.of(15.17),
                    Feet.of(13.235),
                    Feet.of(6.0)),
                () -> new Pose2d(
                    Meters.convertFrom(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseXFt", 0),
                        Feet),
                    Meters.convertFrom(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseYFt", 0),
                        Feet),
                    Rotation2d.fromDegrees(
                        SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotPoseRotationDegrees", 0))),
                () -> new VelocityVector(
                    FeetPerSecond
                        .of(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityXFtps", 0)),
                    FeetPerSecond
                        .of(SmartDashboard.getNumber("frc3620/ShotCalculator/TestInputs/RobotVelocityYFtps", 0))))
                .in(RPM));
      }
    }.withName("Calculate Test Shot").ignoringDisable(true));
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
