package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.AutoAlgae;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ClimbBurst;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.CoralIntakeFromGround;
import frc.robot.commands.CoralIntakeFromGroundUpL1;
import frc.robot.commands.CoralIntakeFromGroundUpSimple;
import frc.robot.commands.CoralIntakeMoveL1;
import frc.robot.commands.CoralIntakeShootL1;
import frc.robot.commands.CoralIntakeSimple;
import frc.robot.commands.CoralShoot;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.VibrateXbox;
import frc.robot.commands.WaypointAlign;
import frc.robot.commands.auto.BargeLeftAuto;
import frc.robot.commands.auto.BargeRightAuto;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.legacy.CoralIntakeFromGroundToggled;
import frc.robot.commands.legacy.CoralIntakeFromSource;
import frc.robot.commands.legacy.ElevatorAlgaeToggled;
import frc.robot.commands.legacy.ElevatorReefToggled;
import frc.robot.commands.legacy.GroundAlgaeToggled;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.ManualElevatorArm;
import frc.robot.control.*;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import frc.robot.subsystems.io.real.ClimberReal;
import frc.robot.subsystems.io.real.CoralIntakeReal;
import frc.robot.subsystems.io.real.ElevatorArmReal;
import frc.robot.subsystems.io.real.ElevatorHeadReal;
import frc.robot.subsystems.io.real.ElevatorReal;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.subsystems.io.sim.ElevatorHeadSim;
import frc.robot.subsystems.io.sim.ElevatorSim;
import frc.robot.utils.AutoPath;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.HumanPlayerSimulation;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.RepeatConditionallyCommand;
import frc.robot.utils.RuckigWarmup;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings({"java:S1444", "java:S1104", "java:S3010"})
public final class RobotContainer {

    public static final double ROBOT_PERIODIC = 0.02;
    public static final int CONTROL_JOYSTICK_PORT = 2;
    public static final int CONTROL_XBOX_PORT = 0;

    public static final double INVALID_COMMAND_VIBRATE_TIME = 0.1;

    public static final double ELEVATOR_LOCK_REEF_DISTANCE = 0.2;
    public static final double ELEVATOR_LOCK_REEF_ANGLE_DIFF = 80; /* degrees */

    public static final double AUTO_ALIGN_FIRST_WAYPOINT_TIMEOUT = 2.0;
    public static final double AUTO_ALIGN_SECOND_WAYPOINT_TIMEOUT = 1.0;

    // Min time remaining in which we can auto reset encoders if not already reset during autonomous
    // (15 - 13 = 2 seconds at the start of auto)
    public static final double FMS_AUTO_RESET_ENCODERS_MIN_TIME = 13;

    /**
     * The time remaining in the match after which the endgame starts and it is time to climb
     */
    public static final double ENDGAME_CLIMB_TIME = 30.0; // seconds

    public static final int[] NON_REEF_TAG_IDS = new int[] {1, 2, 3, 4, 5, 12, 13, 14, 15, 16};
    public static final double PHOTON_SIM_NOISY_STDDEV_POS = 0.2;
    public static final double PHOTON_SIM_NOISY_STDDEV_ROT = 0.1;

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static Elevator elevator;
    public static ElevatorArm elevatorArm;
    public static ElevatorHead elevatorHead;
    public static Climber climber;
    public static Lights lights;
    public static PowerDistributionPanel pdp;
    public static CoralIntake coralIntake;
    public static RobotModel model;
    public static CoralDetection coralDetection;
    public static VisionSystemSim visionSim;
    public static HumanPlayerSimulation humanPlayerSimulation;

    private static Alert noEncoderResetAlert;

    private static RobotContainer instance;

    private RobotContainer() {
        initialize();
    }

    public static RobotContainer createAndInitialize() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private static void initialize() {
        noEncoderResetAlert = new Alert("Encoders not reset!", AlertType.kError);

        try {
            drivetrain = new Drivetrain();
        } catch (InvalidConfigException e) {
            ConsoleLogger.logError("Could not load drivetrain config!", e);
            return;
        }

        if (Constants.RobotState.getMode() == Mode.REAL) {
            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorReal(ROBOT_PERIODIC));
            elevatorArm = new ElevatorArm(new ElevatorArmReal(ROBOT_PERIODIC));
            elevatorHead = new ElevatorHead(new ElevatorHeadReal(ROBOT_PERIODIC));
            coralIntake = new CoralIntake(new CoralIntakeReal(ROBOT_PERIODIC));
            climber = new Climber(new ClimberReal(ROBOT_PERIODIC));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
        } else {
            if (Constants.RobotState.VISION_SIMULATION_MODE.isPhotonSim()) {
                visionSim = new VisionSystemSim("main");

                if (Constants.RobotState.VISION_SIMULATION_MODE
                        == Constants.RobotState.VisionSimulationMode.PHOTON_SIM_INACCURATE) {
                    AprilTagFieldLayout noisyTagLayout = SimpleMath.addNoiseToAprilTagFieldLayout(
                            Constants.Game.APRILTAG_LAYOUT,
                            NON_REEF_TAG_IDS,
                            PHOTON_SIM_NOISY_STDDEV_POS,
                            PHOTON_SIM_NOISY_STDDEV_POS,
                            0,
                            0,
                            0,
                            PHOTON_SIM_NOISY_STDDEV_ROT);
                    Logger.recordOutput(
                            "PhotonSimTagLayout/Poses",
                            noisyTagLayout.getTags().stream().map(t -> t.pose).toArray(Pose3d[]::new));
                    Logger.recordOutput(
                            "PhotonSimTagLayout/IDs",
                            noisyTagLayout.getTags().stream()
                                    .mapToInt(t -> t.ID)
                                    .toArray());

                    visionSim.addAprilTags(noisyTagLayout);
                } else {
                    visionSim.addAprilTags(Constants.Game.APRILTAG_LAYOUT);
                }
            }

            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorSim(ROBOT_PERIODIC));
            elevatorArm = new ElevatorArm(new ElevatorArmSim(ROBOT_PERIODIC));
            elevatorHead = new ElevatorHead(new ElevatorHeadSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            coralIntake = new CoralIntake(new CoralIntakeSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            climber = new Climber(new ClimberSim(ROBOT_PERIODIC));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
            humanPlayerSimulation = new HumanPlayerSimulation();
        }

        model = new RobotModel();

        // Sets up auto path
        AutoPath.initialize();
        // Warmup Ruckig
        RuckigWarmup.warmupCommand().schedule();

        DashboardUI.Autonomous.setupAutoChooser(BargeLeftAuto::new, BargeRightAuto::new);
        PlannedAuto.setAutoSupplier(DashboardUI.Autonomous::getAutoChooser);

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(
                new JoystickXboxSimple(CONTROL_JOYSTICK_PORT, CONTROL_XBOX_PORT),
                new JoystickXbox(
                        CONTROL_JOYSTICK_PORT, CONTROL_XBOX_PORT) /*, new XboxSimpleBackup(CONTROL_XBOX_PORT)*/);

        configureLegacyTriggers();
        configureTriggers();

        elevator.setDefaultCommand(new ManualElevator());
        elevatorArm.setDefaultCommand(new ManualElevatorArm());

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
        }

        SmartDashboard.putNumber(
                "Additional Reef Offset Left", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT);
        SmartDashboard.putNumber(
                "Additional Reef Offset Right", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT);
        SmartDashboard.putNumber(
                "Additional Reef Offset Back", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_BACK);
    }

    public static void teleopInit() {
        DriverStationUtils.teleopInit();
    }

    public static void disabledInit() {
        /* nothing to do */
    }

    public static void autonomousInit() {
        // FMS only reset encoders when enabling autonomous for the first time if not already reset (safety measure)
        // Time limit is so that if the robot restarts during the auto we don't want to reset encoders
        // Don't want to autoreset in teleop because motors keep their positions if you don't power cycle
        if (DriverStation.isFMSAttached()
                && DriverStation.getMatchTime() >= FMS_AUTO_RESET_ENCODERS_MIN_TIME
                && noEncoderResetAlert.get()) {
            resetEncoders();
        }
    }

    public static void disabledExit() {
        // For the generic any enable only reset if not connected to FMS
        // because we don't want to reset encoders if robot restarts in the middle of the match
        // since the motors keep their positions if you don't power cycle
        // for FMS handling see autonomousInit
        // Reset encoders when enabling if not already reset (safety measure)
        if (!DriverStation.isFMSAttached() && noEncoderResetAlert.get()) {
            resetEncoders();
        }
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureLegacyTriggers() {
        configureElevatorTriggers(RobotContainer::checkElevatorLock);
        configureCoralTriggers();
        configureAlgaeTriggers(RobotContainer::checkAlgaeLock);
        configureAutoAlignTrigger();
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static boolean checkElevatorLock() {
        boolean atBottom = elevator.getNearestHeight() == ElevatorHeight.INTAKE
                || elevator.getNearestHeight() == ElevatorHeight.BOTTOM;
        boolean atL4 = elevator.getNearestHeight() == ElevatorHeight.L4
                || elevator.getNearestHeight() == ElevatorHeight.BARGE_ALGAE;

        Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
        CoralPosition closestReef = IGamePosition.closestTo(robot, CoralPosition.values());

        boolean nearReef = isNearReef(robot, closestReef);

        return DashboardUI.Overview.getControl().isManualOverrideTriggered()
                || (atBottom && elevatorHead.coralReady())
                || (atL4 && !nearReef)
                || (!atBottom && !atL4);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static boolean isNearReef(Pose2d robot, CoralPosition closestReef) {
        return closestReef.getFirstStagePose().getTranslation().getDistance(robot.getTranslation())
                        < ELEVATOR_LOCK_REEF_DISTANCE
                && Math.abs(closestReef
                                .getFirstStagePose()
                                .getRotation()
                                .minus(robot.getRotation())
                                .getDegrees())
                        < ELEVATOR_LOCK_REEF_ANGLE_DIFF;
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static boolean checkAlgaeLock() {
        return DashboardUI.Overview.getControl().isManualOverrideTriggered()
                || !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureElevatorTriggers(BooleanSupplier elevatorLock) {
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL2Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L2));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL3Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L3));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL4Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L4));

        new Trigger(() -> (DashboardUI.Overview.getControl().isElevatorL2Triggered()
                                || DashboardUI.Overview.getControl().isElevatorL3Triggered()
                                || DashboardUI.Overview.getControl().isElevatorL4Triggered())
                        && !elevatorLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureCoralTriggers() {
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralShootTriggered()).onTrue(new CoralShoot());
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralGroundIntakeTriggered())
                .toggleOnTrue(new CoralIntakeFromGroundToggled());

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralSourceIntakeTriggered())
                .onTrue(new CoralIntakeFromSource(true));

        Command coralScoreL1Cmd = createCoralScoreL1Command();
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralIntakeScoreL1Triggered())
                .onTrue(coralScoreL1Cmd.asProxy());
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static Command createCoralScoreL1Command() {
        return Commands.either(
                Commands.either(
                        new CoralIntakeShootL1().asProxy(),
                        new CoralIntakeMoveL1().asProxy(),
                        () -> coralIntake.getState() == CoralIntakeState.L1_DOWN),
                new CoralIntakeFromGroundUpL1()
                        .asProxy()
                        .beforeStarting(() -> CoralIntakeFromGroundToggled.isGoingToL1 = true),
                () -> coralIntake.getState() != CoralIntakeState.GROUND);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAlgaeTriggers(BooleanSupplier algaeLock) {
        new Trigger(() -> DashboardUI.Overview.getControl().isGroundAlgaeTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new GroundAlgaeToggled(ElevatorHeight.GROUND_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeLowTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.LOW_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeHighTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.HIGH_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isScoreAlgaeTriggered() && algaeLock.getAsBoolean())
                .onTrue(new ProcessorScore(true));

        configureAlgaeVibrateXboxTriggers(algaeLock);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAlgaeVibrateXboxTriggers(BooleanSupplier algaeLock) {
        new Trigger(() -> (DashboardUI.Overview.getControl().isGroundAlgaeTriggered()
                                || DashboardUI.Overview.getControl().isElevatorAlgaeLowTriggered())
                        && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeHighTriggered() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
        new Trigger(() -> DashboardUI.Overview.getControl().isScoreAlgaeTriggered() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kLeftRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAutoAlignTrigger() {
        new Trigger(() -> DashboardUI.Overview.getControl().isAutoAlignTriggered())
                .whileTrue(Commands.defer(
                        () -> WaypointAlign.align(
                                ReefAlign.generateWaypointsClosest(false),
                                0,
                                1,
                                true,
                                new Double[] {AUTO_ALIGN_FIRST_WAYPOINT_TIMEOUT, AUTO_ALIGN_SECOND_WAYPOINT_TIMEOUT},
                                AutoControlModifier.getDefault(),
                                AutoUtils::getCurrentDrivetrainKinematicState),
                        Set.of(drivetrain)));
    }

    private static void configureTriggers() {

        // Command to kill robot
        new Trigger(() -> DashboardUI.Overview.getControl().isKillTriggered())
                .whileTrue(new KillSpecified(drivetrain, elevatorHead, coralIntake, climber)
                        .alongWith(new InstantCommand(
                                () -> CommandScheduler.getInstance().cancelAll())));

        new Trigger(() -> DashboardUI.Overview.getControl().isClimbTriggered())
                .onTrue(Commands.either(
                                new ClimbMove(ClimberState.CLIMB),
                                new ClimbMove(ClimberState.EXTEND),
                                () -> climber.getCurrentState() == ClimberState.EXTEND)
                        .alongWith(new InstantCommand(() -> Elastic.selectTab("Climb"))));

        new Trigger(() -> DashboardUI.Overview.getControl().isClimbBurstTriggered())
                .onTrue(new ClimbBurst().onlyIf(() -> climber.getCurrentState() == ClimberState.CLIMB));

        new Trigger(() -> DriverStationUtils.getTeleopMatchTime().orElse(Double.MAX_VALUE) <= ENDGAME_CLIMB_TIME)
                .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1.0).withTimeout(2.0));

        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().isPoseResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
        new Trigger(() -> DashboardUI.Overview.getControl().isLimelightResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision));
        new Trigger(DashboardUI.Autonomous::isResetLocationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
        new Trigger(DashboardUI.Autonomous::isLimelightRotationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision).ignoringDisable(true));
        new Trigger(DashboardUI.Autonomous::isEncoderResetPressed)
                .onTrue(new InstantCommand(RobotContainer::resetEncoders).ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(new CoralIntakeFromGround())
                .onFalse(Commands.either(
                                new CoralIntakeFromGroundUpL1(),
                                new CoralIntakeFromGroundUpSimple()
                                        .handleInterrupt(() -> CoralIntakeFromGroundUpSimple.setRunning(false))
                                        .andThen(new CoralIntakeSimple(false)
                                                .finallyDo(() -> {
                                                    CoralIntakeSimple.setRunning(false);
                                                    CoralIntakeFromGroundUpSimple.setRunning(false);
                                                })
                                                .onlyIf(() -> !CoralIntakeSimple.isRunning())),
                                () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                        == ReefLevelSwitchValue.L1)
                        .onlyWhile(() -> elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                                || !DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()));

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralSourceIntakeAutoTriggered()
                        && !DriverStation.isAutonomous())
                .debounce(1.0, DebounceType.kFalling)
                .whileTrue(new RepeatConditionallyCommand(
                                new CoralIntakeSimple(true)
                                        .finallyDo(() -> {
                                            CoralIntakeSimple.setRunning(false);
                                            RobotContainer.elevatorHead.set(CoralShooterStates.OFF);
                                            RobotContainer.coralIntake.set(CoralIntakeState.UP);
                                        })
                                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                                        .asProxy()
                                        .onlyIf(() -> !CoralIntakeSimple.isRunning()
                                                && !CoralIntakeFromGroundUpSimple.isRunning()
                                                && !AutoAlgae.isRunning()),
                                () -> !RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL_CERTAIN)
                                        && !DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()
                                        && DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                                != ReefLevelSwitchValue.L1,
                                false)
                        .ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().isReefAlgaeSimpleTriggered()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(Commands.either(
                        Commands.defer(
                                () -> new ScheduleCommand(new AutoAlgae(IGamePosition.closestTo(
                                                SimpleMath.integrateChassisSpeeds(
                                                        RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                        RobotContainer.drivetrain.getChassisSpeeds(),
                                                        Constants.Control.SCORE_TARGET_LOOKAHEAD),
                                                AlgaePosition.values()))
                                        .finallyDo(AutoAlgae::stopRunning)
                                        .asProxy()),
                                Set.of()),
                        new InstantCommand(AutoAlgae::performCancel),
                        () -> !AutoAlgae.isRunning()));

        new Trigger(() -> DashboardUI.Overview.getControl().isAutoScoreTriggered())
                .onTrue(Commands.either(
                        Commands.deferredProxy(() -> new AutoScore(
                                IGamePosition.closestTo(
                                        SimpleMath.integrateChassisSpeeds(
                                                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                RobotContainer.drivetrain.getChassisSpeeds(),
                                                Constants.Control.SCORE_TARGET_LOOKAHEAD), // where the robot will be in
                                        // LOOKAHEAD seconds
                                        CoralPosition.values()),
                                DashboardUI.Overview.getControl()
                                        .getReefLevelSwitchValue()
                                        .toCoralLevel())),
                        new ProcessorScore(false).asProxy(),
                        () -> elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                                || (DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                                == ReefLevelSwitchValue.L1
                                        && !elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        if (SysIdManager.getProvider().isEnabled()) {
            return SysIdManager.getProvider().createCommand();
        }

        return PlannedAuto.getAutoCommand();
    }

    public static void robotPeriodic() {
        Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT = SmartDashboard.getNumber(
                "Additional Reef Offset Left", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT);
        Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT = SmartDashboard.getNumber(
                "Additional Reef Offset Right", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT);
        Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_BACK = SmartDashboard.getNumber(
                "Additional Reef Offset Back", Constants.Game.CoralPosition.ADDITIONAL_REEF_SEGMENT_OFFSET_BACK);
    }

    public static void simulationPeriodic() {
        updateSimulationBattery(drivetrain, elevator, elevatorHead, coralIntake);
        if (Constants.RobotState.VISION_SIMULATION_MODE.isPhotonSim()) {
            visionSim.update(model.getRobot());
        }
    }

    public static void updateSimulationBattery(PoweredSubsystem... subsystems) {
        double[] currents = new double[subsystems.length];
        for (int i = 0; i < subsystems.length; i++) {
            currents[i] = subsystems[i].getCurrentDrawAmps();
        }
    }

    public static void resetEncoders() {
        climber.resetEncoders();
        elevator.resetEncoders();
        elevatorArm.resetEncoders();
        coralIntake.resetEncoders();

        noEncoderResetAlert.set(false);
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Encoders reset!", "Successfully reset arm encoders."));
    }

    /** frees up all hardware allocations */
    public static void close() throws Exception {
        drivetrain.close();
        poseSensorFusion.close();
        elevator.close();
        elevatorArm.close();
        elevatorHead.close();
        coralIntake.close();
        pdp.close();
    }
}
