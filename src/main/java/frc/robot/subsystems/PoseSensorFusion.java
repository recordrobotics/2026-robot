package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.utils.camera.CameraType;
import frc.robot.utils.camera.IVisionCamera;
import frc.robot.utils.camera.LimelightCamera;
import frc.robot.utils.camera.PhotonVisionCamera;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

public class PoseSensorFusion extends ManagedSubsystemBase {

    public static final double MAX_MEASUREMENT_STD_DEVS = 9_999_999;

    private static final double L1_STD_MULTIPLIER = 1.0;
    private static final double SOURCE_STD_MULTIPLIER = 1.0;

    private static final double ISPE_STD_DEV = 0.7;

    private static final double MIN_TXTY_FPS = 8.0;
    private static final double MAX_TXTY_TIME = 1.0 / MIN_TXTY_FPS;

    private static final double MAX_TXTY_DISTANCE_TO_LAST_POSE = 0.3;
    private static final double MAX_TXTY_DISTANCE_TO_LAST_TAG = 0.8;

    public final NavSensor nav;

    private SwerveDrivePoseEstimator poseFilter;
    private IndependentSwervePoseEstimator independentPoseEstimator;

    private LimelightCamera leftCamera = new LimelightCamera(
            Constants.Limelight.LIMELIGHT_LEFT_NAME, CameraType.LIMELIGHT_3G, Constants.Limelight.ROBOT_TO_CAMERA_LEFT);
    private LimelightCamera centerCamera = new LimelightCamera(
            Constants.Limelight.LIMELIGHT_CENTER_NAME,
            CameraType.LIMELIGHT_2,
            Constants.Limelight.ROBOT_TO_CAMERA_CENTER);

    private PhotonVisionCamera l1Camera = new PhotonVisionCamera(
            Constants.PhotonVision.PHOTON_L1_NAME,
            CameraType.SVPRO_GLOBAL_SHUTTER,
            Constants.PhotonVision.ROBOT_TO_CAMERA_L1,
            L1_STD_MULTIPLIER);
    private PhotonVisionCamera sourceCamera = new PhotonVisionCamera(
            Constants.PhotonVision.PHOTON_SOURCE_NAME,
            CameraType.SVPRO_GLOBAL_SHUTTER,
            Constants.PhotonVision.ROBOT_TO_CAMERA_SOURCE,
            SOURCE_STD_MULTIPLIER);

    private final Set<IVisionCamera> cameras = Set.of(leftCamera, centerCamera, l1Camera, sourceCamera);

    private ConcurrentSkipListSet<DeferredPoseEstimation> deferredPoseEstimations =
            new ConcurrentSkipListSet<>((a, b) -> Double.compare(a.timestampSeconds, b.timestampSeconds));

    private boolean trustLimelightLeft;
    private boolean trustLimelightCenter;
    private boolean useOPI;
    private boolean useVision;

    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private Future<?> calculationFuture = null;

    private double updateTimestamp;
    private Rotation2d updateNav;
    private SwerveModulePosition[] updatePositions;

    private double lastMostCommonTxtyTime = 0;
    private int lastMostCommonTxtyId = -1;
    private Pose2d lastMostCommonTxtyRobotPose = new Pose2d();

    public PoseSensorFusion() {
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
                getModulePositions(),
                DashboardUI.Autonomous.getStartingLocation().getPose());

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

        SmartDashboard.putBoolean("Autonomous/TrustLimelightLeft", true);
        SmartDashboard.putBoolean("Autonomous/TrustLimelightCenter", true);
        SmartDashboard.putBoolean("Autonomous/UseOPI", true);
        SmartDashboard.putBoolean("Autonomous/UseVision", true);
    }

    public record DeferredPoseEstimation(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        deferredPoseEstimations.add(
                new DeferredPoseEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    }

    public void startCalculation() {
        deferredPoseEstimations.clear();

        trustLimelightLeft = SmartDashboard.getBoolean("Autonomous/TrustLimelightLeft", false);
        trustLimelightCenter = SmartDashboard.getBoolean("Autonomous/TrustLimelightCenter", false);
        useOPI = SmartDashboard.getBoolean("Autonomous/UseOPI", false);
        useVision = SmartDashboard.getBoolean("Autonomous/UseVision", true);

        calculationFuture = executor.submit(this::calculationLoop);
    }

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
            if (useVision) {
                poseFilter.addVisionMeasurement(
                        estimation.visionRobotPoseMeters,
                        estimation.timestampSeconds,
                        estimation.visionMeasurementStdDevs);
            }
        }

        updateDashboard();

        leftCamera.logValues("Left");
        centerCamera.logValues("Center");
        l1Camera.logValues("L1");
        sourceCamera.logValues("Source");

        Logger.recordOutput("SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
        Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
        Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());

        Logger.recordOutput("MostCommonTXTYId", lastMostCommonTxtyId);

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

    public void calculationLoop() {
        updateTimestamp = Timer.getTimestamp();
        updateNav = nav.getAdjustedAngle();
        updatePositions = getModulePositions();

        cameras.stream().forEach(IVisionCamera::updateEstimation);

        updateMostCommonTXTYId();

        addCameraVisionMeasurement(trustLimelightLeft, lastMostCommonTxtyId, leftCamera);
        addCameraVisionMeasurement(trustLimelightCenter, lastMostCommonTxtyId, centerCamera);

        if (useOPI) {
            addCameraVisionMeasurement(true, lastMostCommonTxtyId, l1Camera);
            addCameraVisionMeasurement(true, lastMostCommonTxtyId, sourceCamera);
        }

        updateIndependentPoseEstimator();
    }

    /**
     * Updates the last known most common TXTY ID and time based on the current most common TXTY ID
     */
    private void updateMostCommonTXTYId() {
        int mostCommonTxtyId = getMostCommonTXTYId();

        if (shouldUpdateTXTYId(mostCommonTxtyId)) {
            lastMostCommonTxtyId = mostCommonTxtyId;
            lastMostCommonTxtyTime = Timer.getTimestamp();
            if (mostCommonTxtyId != -1) {
                lastMostCommonTxtyRobotPose = getEstimatedPosition();
            }
        }
    }

    /**
     * Determines whether the last known most common TXTY ID should still be considered visible based on
     * <ul>
     *  <li>Distance to where the tag was last seen
     *  <li>Distance to the tag's known position
     * </ul>
     * @return true if the last known most common TXTY ID should be considered visible, false otherwise
     */
    private boolean shouldLastMostCommonTXTYIdBeVisible() {
        // we are close to where we last saw the tag but don't see it anymore, likely lost tracking
        // don't use potentially unreliable vision from PNP solver far away from tag

        Pose2d currentPose = getEstimatedPosition();
        double distanceToWhereTagLastSeen =
                currentPose.getTranslation().getDistance(lastMostCommonTxtyRobotPose.getTranslation());

        if (distanceToWhereTagLastSeen > MAX_TXTY_DISTANCE_TO_LAST_POSE) {
            return false;
        }

        Optional<Pose3d> tagPose = Constants.Game.APRILTAG_LAYOUT.getTagPose(lastMostCommonTxtyId);
        if (tagPose.isPresent()) {
            // we are close to the tag but don't see it anymore, likely lost tracking
            // don't use potentially unreliable vision from PNP solver far away from tag

            double distanceToTag = currentPose
                    .getTranslation()
                    .getDistance(tagPose.get().toPose2d().getTranslation());

            if (distanceToTag > MAX_TXTY_DISTANCE_TO_LAST_TAG) {
                return false;
            }
        }

        return true;
    }

    /**
     * Determines whether to update the last known most common TXTY ID and time based on:
     * <ul>
     *  <li>If there is no last known ID (the delay is only on falling edge)
     *  <li>If the most common ID is the same as the last known (update the last seen time)
     *  <li>If the tracked tag is lost but should still be visible (odometry is likely more accurate than far away PNP pose)
     *  <li>If the last known ID is stale (exceeded max time without being seen)
     * </ul>
     * @param mostCommonTxtyId the currently most common TXTY ID or -1 if none are detected
     * @return true if the last known most common TXTY ID should be updated, false otherwise
     */
    @SuppressWarnings("java:S1126") // last return is necessary for clarity
    private boolean shouldUpdateTXTYId(int mostCommonTxtyId) {
        if (lastMostCommonTxtyId == -1) return true; // always update if we don't have a last known ID

        if (mostCommonTxtyId == lastMostCommonTxtyId)
            return true; // always update if the most common ID is the same as last known

        if (mostCommonTxtyId == -1 && shouldLastMostCommonTXTYIdBeVisible())
            return false; // don't update if we lost tracking of the last known ID and it should be visible

        if (lastMostCommonTxtyTime + MAX_TXTY_TIME < Timer.getTimestamp())
            return true; // update if the last known ID is stale

        return false;
    }

    /**
     * Finds the most common TXTY ID among all cameras with valid estimates
     * @return the most common TXTY ID or -1 if none are detected
     */
    private int getMostCommonTXTYId() {
        Map<Integer, Integer> txtyTagsCount = new HashMap<>();
        for (IVisionCamera camera : cameras) {
            if (camera.getNumTags() > 0
                    && camera.getMeasurementStdDevs() < MAX_MEASUREMENT_STD_DEVS
                    && camera.getUnsafeEstimate().isTXTY()) {
                int id = camera.getUnsafeEstimate().txtyId();
                txtyTagsCount.put(id, txtyTagsCount.getOrDefault(id, 0) + 1);
            }
        }

        int mostCommonTxtyId = -1;
        int mostCommonTxtyCount = 0;
        for (Map.Entry<Integer, Integer> entry : txtyTagsCount.entrySet()) {
            if (entry.getValue() > mostCommonTxtyCount) {
                mostCommonTxtyId = entry.getKey();
                mostCommonTxtyCount = entry.getValue();
            }
        }

        return mostCommonTxtyId;
    }

    /**
     * Updates the independent pose estimator and uses it to correct the main pose estimator when no vision is available
     */
    private void updateIndependentPoseEstimator() {
        if (cameras.stream().anyMatch(v -> v.hasVision())) {
            // when vision is correcting the pose, have that override the independent pose estimator
            independentPoseEstimator.reset(getEstimatedPosition());
            independentPoseEstimator.update(getEstimatedPosition().getRotation());
        } else {
            independentPoseEstimator.update(getEstimatedPosition().getRotation());

            // when no vision use independent pose estimator to correct pose
            addVisionMeasurement(
                    independentPoseEstimator.getEstimatedRobotPose(),
                    Timer.getTimestamp(),
                    VecBuilder.fill(ISPE_STD_DEV, ISPE_STD_DEV, MAX_MEASUREMENT_STD_DEVS));
        }
    }

    /**
     * Adds a vision measurement from a camera to the pose filter with measurement rejection if necessary
     * <ul>
     *  <li>If there is no targeted TXTY tag, always add the measurement
     *  <li>If the camera's estimate is TXTY and matches the most common TXTY ID, add the measurement
     *  <li>Otherwise, do not add the measurement
     * </ul>
     * @param trust whether to trust the camera's rotation
     * @param mostCommonTxtyId the currently detected TXTY tag or -1 if none are detected
     * @param camera the camera to add the measurement from
     */
    private void addCameraVisionMeasurement(boolean trust, int mostCommonTxtyId, IVisionCamera camera) {
        if (mostCommonTxtyId == -1
                || (camera.getUnsafeEstimate().isTXTY()
                        && camera.getUnsafeEstimate().txtyId() == mostCommonTxtyId)) {
            camera.addVisionMeasurement(
                    trust,
                    getEstimatedPositionAt(camera.getUnsafeEstimate().timestampSeconds())
                            .orElse(null));
        }
    }

    private void updateDashboard() {
        for (IVisionCamera camera : cameras) {
            DashboardUI.Autonomous.setVisionPose(
                    camera.getName(), camera.getUnsafeEstimate().pose());
        }

        SmartDashboard.putNumber(
                "pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
        DashboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
    }

    private static SwerveModulePosition[] getModulePositions() {
        return RobotContainer.drivetrain.getModulePositions();
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

    /** Similar to resetPose but adds an argument for the initial pose */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
        independentPoseEstimator.reset(pose);
    }

    /** Resets the field relative position of the robot (mostly for testing). */
    public void resetStartingPose() {
        setToPose(DashboardUI.Autonomous.getStartingLocation().getPose());
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.drivetrain
                    .getSwerveDriveSimulation()
                    .setSimulationWorldPose(
                            DashboardUI.Autonomous.getStartingLocation().getPose());
        }
    }

    public void resetToVision() {
        cameras.stream()
                .filter(cam -> cam.getNumTags() > 0)
                .sorted((a, b) -> Double.compare(
                        a.getUnsafeEstimate().avgTagDist(),
                        b.getUnsafeEstimate().avgTagDist()))
                .findFirst()
                .ifPresentOrElse(
                        camera -> setToPose(camera.getUnsafeEstimate().pose()),
                        () -> ConsoleLogger.logWarning("No camera has vision!"));
    }

    /**
     * Resets the pose to face elevator away from driverstation, while keeping translation the same
     */
    public void resetDriverPose() {
        Pose2d current = getEstimatedPosition();
        setToPose(new Pose2d(
                current.getTranslation(),
                DriverStationUtils.getCurrentAlliance() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d(0)));
    }

    @Override
    public void close() throws Exception {
        nav.close();
    }
}
