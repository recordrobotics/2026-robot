package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.real.NavSensorReal;
import frc.robot.subsystems.io.sim.NavSensorSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.IndependentSwervePoseEstimator;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.Cameras;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.poseestimation.PoseEstimationCamera;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for fusing multiple pose sensors (odometry, IMU, vision) into a single robot pose estimation
 */
public final class PoseSensorFusion extends ManagedSubsystemBase {

    /**
     * The maximum standard deviation for a vision measurement (used to indicate an untrusted measurement)
     */
    public static final double MAX_MEASUREMENT_STD_DEVS = 9_999_999;

    /**
     * The standard deviation to use for the independent swerve pose estimator corrections
     */
    private static final double ISPE_STD_DEV = 0.7;

    /**
     * The NAV sensor used for orientation and acceleration data
     */
    public final NavSensor nav;

    /**
     * The swerve drive pose estimator used for fusing odometry and vision measurements
     */
    private SwerveDrivePoseEstimator poseFilter;
    /**
     * The independent swerve pose estimator used for better odometry when no vision is available
     */
    private IndependentSwervePoseEstimator independentPoseEstimator;

    /**
     * The cameras used for vision measurements
     */
    private final Set<PoseEstimationCamera> cameras = Set.of(
            Cameras.createLimelightPoseEstimationCamera(
                    Constants.Vision.LEFT_FRONT_NAME,
                    PhysicalCamera.LIMELIGHT_3G,
                    Constants.Vision.ROBOT_TO_CAMERA_LEFT_FRONT),
            Cameras.createLimelightPoseEstimationCamera(
                    Constants.Vision.LEFT_BACK_NAME,
                    PhysicalCamera.LIMELIGHT_2,
                    Constants.Vision.ROBOT_TO_CAMERA_LEFT_BACK),
            Cameras.createPhotonVisionPoseEstimationCamera(
                    Constants.Vision.RIGHT_FRONT_NAME,
                    PhysicalCamera.SVPRO_GLOBAL_SHUTTER,
                    Constants.Vision.ROBOT_TO_CAMERA_RIGHT_FRONT),
            Cameras.createPhotonVisionPoseEstimationCamera(
                    Constants.Vision.RIGHT_BACK_NAME,
                    PhysicalCamera.SVPRO_GLOBAL_SHUTTER,
                    Constants.Vision.ROBOT_TO_CAMERA_RIGHT_BACK),
            Cameras.createLimelightPoseEstimationCamera(
                    Constants.Vision.TURRET_NAME, PhysicalCamera.LIMELIGHT_4, Transform3d.kZero));

    /**
     * The deferred pose estimations to be added at the end of the calculation phase
     */
    private ConcurrentSkipListSet<DeferredPoseEstimation> deferredPoseEstimations =
            new ConcurrentSkipListSet<>((a, b) -> Double.compare(a.timestampSeconds, b.timestampSeconds));

    /**
     * Executor service for asynchronous calculations
     */
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    /**
     * Future for the current calculation task
     */
    private Future<?> calculationFuture = null;

    /**
     * Last update timestamp
     */
    private double updateTimestamp;
    /**
     * Last update nav angle
     */
    private Rotation2d updateNav;
    /**
     * Last update swerve module positions
     */
    private SwerveModulePosition[] updatePositions;

    /**
     * The TXTY ID to use for vision measurements (-1 for disabled)
     */
    private int targetTXTYId = -1;

    /**
     * Creates a new PoseSensorFusion subsystem
     * @param initialPose the initial pose of the robot on the field
     */
    public PoseSensorFusion(Pose2d initialPose) {
        nav = new NavSensor(
                Constants.RobotState.getMode() == Mode.REAL
                        ? new NavSensorReal()
                        : new NavSensorSim(RobotContainer.drivetrain
                                .getSwerveDriveSimulation()
                                .getGyroSimulation()));
        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                RobotContainer.drivetrain.getKinematics(),
                nav.getAdjustedAngle(),
                RobotContainer.drivetrain.getModulePositions(),
                initialPose);

        independentPoseEstimator = new IndependentSwervePoseEstimator(
                getEstimatedPosition(),
                new SwerveModule[] {
                    RobotContainer.drivetrain.getFrontLeftModule(),
                    RobotContainer.drivetrain.getFrontRightModule(),
                    RobotContainer.drivetrain.getBackLeftModule(),
                    RobotContainer.drivetrain.getBackRightModule()
                },
                new Translation2d[] {
                    Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
                    Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
                    Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
                    Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION
                });
    }

    public record DeferredPoseEstimation(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

    /**
     * Adds a vision measurement to be processed at the end of the calculation phase
     * @param visionRobotPoseMeters the vision-based robot pose measurement
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        deferredPoseEstimations.add(
                new DeferredPoseEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    }

    /**
     * Starts the asynchronous calculation phase
     */
    public void startCalculation() {
        deferredPoseEstimations.clear();
        calculationFuture = executor.submit(this::performCalculation);
    }

    /**
     * Waits for the end of the calculation and applies all updates
     */
    public void endCalculation() {
        if (updatePositions != null) {
            poseFilter.updateWithTime(updateTimestamp, updateNav, updatePositions);
        }

        if (calculationFuture != null) {
            try {
                calculationFuture.get();
            } catch (ExecutionException e) {
                ConsoleLogger.logError(e);
            } catch (InterruptedException e) {
                ConsoleLogger.logError(e);
                Thread.currentThread().interrupt();
            }
        }

        while (!deferredPoseEstimations.isEmpty()) {
            DeferredPoseEstimation estimation = deferredPoseEstimations.pollFirst();
            poseFilter.addVisionMeasurement(
                    estimation.visionRobotPoseMeters, estimation.timestampSeconds, estimation.visionMeasurementStdDevs);
        }

        logValues();
    }

    /**
     * Logs values to the logger
     */
    private void logValues() {
        DashboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());

        cameras.stream().forEach(GenericCamera::logValues);

        Logger.recordOutput("SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
        Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
        Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());

        Logger.recordOutput("TargetTXTYId", targetTXTYId);

        Logger.recordOutput(
                "IntegratedPose",
                SimpleMath.integrateChassisSpeeds(
                        getEstimatedPosition(),
                        RobotContainer.drivetrain.getChassisSpeeds(),
                        Constants.Control.SCORE_TARGET_LOOKAHEAD));

        Logger.recordOutput("NAV/Pitch", nav.getPitch());
        Logger.recordOutput("NAV/Roll", nav.getRoll());
    }

    @Override
    public void periodicManaged() {
        /* logic is asynchronous */
    }

    /**
     * Performs the asynchronous calculation for the pose sensor fusion
     */
    public void performCalculation() {
        updateTimestamp = Timer.getTimestamp();
        updateNav = nav.getAdjustedAngle();
        updatePositions = RobotContainer.drivetrain.getModulePositions();

        cameras.stream().forEach(GenericCamera::periodic);

        /* perform any multi-camera logic like txty selection (currently manual) */

        Pose2d currentEstimate = getEstimatedPosition();
        cameras.stream().forEach(camera -> camera.addVisionMeasurements(this, currentEstimate, targetTXTYId));

        updateIndependentPoseEstimator();
    }

    /**
     * Updates the independent pose estimator and uses it to correct the main pose estimator when no vision is available
     */
    private void updateIndependentPoseEstimator() {
        Pose2d estimatedPose = getEstimatedPosition();
        if (cameras.stream().anyMatch(v -> v.hasVision())) {
            // when vision is correcting the pose, have that override the independent pose estimator
            independentPoseEstimator.reset(estimatedPose);
            independentPoseEstimator.update(estimatedPose.getRotation());
        } else {
            independentPoseEstimator.update(estimatedPose.getRotation());

            // when no vision use independent pose estimator to correct pose
            addVisionMeasurement(
                    independentPoseEstimator.getEstimatedRobotPose(),
                    Timer.getTimestamp(),
                    VecBuilder.fill(ISPE_STD_DEV, ISPE_STD_DEV, MAX_MEASUREMENT_STD_DEVS));
        }
    }

    /**
     * Gets the estimated position of the robot on the field at the current time
     * @return the estimated position of the robot on the field
     */
    @AutoLogLevel(key = "Odometry/Robot", level = Level.REAL)
    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    /**
     * Gets the estimated position at a specific timestamp or an empty optional if the timestamp is out of range
     * @param timestamp the timestamp to get the estimated position at (see {@link Timer#getTimestamp()})
     * @return an optional containing the estimated position at the specified timestamp, or empty if out of range
     */
    public Optional<Pose2d> getEstimatedPositionAt(double timestamp) {
        return poseFilter.sampleAt(timestamp);
    }

    /** Resets the robot's pose to the specified pose (if in simulation also resets simulation pose) */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(nav.getAdjustedAngle(), RobotContainer.drivetrain.getModulePositions(), pose);
        independentPoseEstimator.reset(pose);
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.drivetrain
                    .getSwerveDriveSimulation()
                    .setSimulationWorldPose(
                            DashboardUI.Autonomous.getStartingLocation().getPose());
        }
    }

    /**
     * Aligns the robot's rotation to face the robot in the same direction as the driver station
     */
    public void alignRotationWithDriverStation() {
        Pose2d current = getEstimatedPosition();
        setToPose(new Pose2d(
                current.getTranslation(),
                DriverStationUtils.getCurrentAlliance() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d(0)));
    }

    /**
     * Closes the subsystem and releases any resources
     */
    @Override
    public void close() throws Exception {
        nav.close();
    }
}
