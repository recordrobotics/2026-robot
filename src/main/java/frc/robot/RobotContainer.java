package frc.robot;

import com.google.common.primitives.ImmutableIntArray;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotState.Mode;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.VibrateXbox;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.io.real.IntakeReal;
import frc.robot.subsystems.io.real.TurretReal;
import frc.robot.subsystems.io.sim.IntakeSim;
import frc.robot.subsystems.io.sim.TurretSim;
import frc.robot.utils.AutoPath;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.RuckigWarmup;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
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

    public static final double AUTO_ALIGN_FIRST_WAYPOINT_TIMEOUT = 2.0;
    public static final double AUTO_ALIGN_SECOND_WAYPOINT_TIMEOUT = 1.0;

    // Min time remaining in which we can auto reset encoders if not already reset during autonomous
    // (15 - 13 = 2 seconds at the start of auto)
    public static final double FMS_AUTO_RESET_ENCODERS_MIN_TIME = 13;

    /**
     * The time remaining in the match after which we should go to climb
     * the FMS plays a sound at 30 seconds, we don't need that much time
     */
    public static final double XBOX_RUMBLE_ENDGAME_TIME = 15.0; // seconds

    public static final ImmutableIntArray NON_HUB_TAG_IDS =
            ImmutableIntArray.of(1, 12, 13, 14, 15, 16, 7, 6, 17, 28, 29, 30, 31, 32, 23, 22);
    public static final double PHOTON_SIM_NOISY_STDDEV_POS = 0.2;
    public static final double PHOTON_SIM_NOISY_STDDEV_ROT = 0.1;

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static PowerDistributionPanel pdp;
    public static Intake intake;
    public static Turret turret;
    public static RobotModel model;
    public static FieldStateTracker fieldStateTracker;
    public static VisionSystemSim visionSim;

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
            intake = new Intake(new IntakeReal(ROBOT_PERIODIC));
            turret = new Turret(new TurretReal(ROBOT_PERIODIC));
        } else {
            if (Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
                visionSim = new VisionSystemSim("main");

                if (Constants.Vision.VISION_SIMULATION_MODE
                        == Constants.Vision.VisionSimulationMode.PHOTON_SIM_INACCURATE) {
                    // simulate misalignment of field elements, assume tags are perfectly placed on field elements
                    AprilTagFieldLayout noisyTagLayout = SimpleMath.addNoiseToAprilTagFieldLayout(
                            Constants.Game.APRILTAG_LAYOUT,
                            NON_HUB_TAG_IDS,
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

            intake = new Intake(new IntakeSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            turret = new Turret(new TurretSim(ROBOT_PERIODIC));
        }

        poseSensorFusion = new PoseSensorFusion(
                DashboardUI.Autonomous.getStartingLocation().getPose());
        pdp = new PowerDistributionPanel();
        fieldStateTracker = new FieldStateTracker();

        model = new RobotModel();

        // Sets up auto path
        AutoPath.initialize();
        // Warmup Ruckig
        CommandScheduler.getInstance().schedule(RuckigWarmup.warmupCommand());

        DashboardUI.Autonomous.setupAutoChooser();
        PlannedAuto.setAutoSupplier(DashboardUI.Autonomous::getAutoChooser);

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(new JoystickXboxSimple(CONTROL_JOYSTICK_PORT, CONTROL_XBOX_PORT));

        configureTriggers();

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
        }
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

    private static void configureTriggers() {
        new Trigger(() -> DashboardUI.Overview.getControl().isIntakePressed())
                .whileTrue(new InstantCommand(() -> intake.setState(Intake.IntakeState.INTAKE)))
                .whileFalse(new InstantCommand(() -> intake.setState(Intake.IntakeState.RETRACTED)));

        // Command to kill robot
        new Trigger(() -> DashboardUI.Overview.getControl().isKillTriggered())
                .whileTrue(new KillSpecified(drivetrain, intake)
                        .alongWith(new InstantCommand(
                                () -> CommandScheduler.getInstance().cancelAll())));

        new Trigger(() -> DriverStationUtils.getTeleopMatchTime().orElse(Double.MAX_VALUE) <= XBOX_RUMBLE_ENDGAME_TIME)
                .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1.0).withTimeout(2.0));

        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().isPoseResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::alignRotationWithDriverStation));
        new Trigger(DashboardUI.Autonomous::isResetLocationPressed)
                .onTrue(new InstantCommand(() -> poseSensorFusion.setToPose(
                                DashboardUI.Autonomous.getStartingLocation().getPose()))
                        .ignoringDisable(true));
        new Trigger(DashboardUI.Autonomous::isEncoderResetPressed)
                .onTrue(new InstantCommand(RobotContainer::resetEncoders).ignoringDisable(true));
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
        // nothing to do here, command scheduler handles everything
    }

    public static void simulationPeriodic() {
        updateSimulationBattery(drivetrain);
        if (Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
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
        intake.resetEncoders();
        turret.resetEncoders();

        noEncoderResetAlert.set(false);
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Encoders reset!", "Successfully reset arm encoders."));
    }

    /** frees up all hardware allocations */
    public static void close() throws Exception {
        drivetrain.close();
        poseSensorFusion.close();
        intake.close();
        pdp.close();
    }
}
