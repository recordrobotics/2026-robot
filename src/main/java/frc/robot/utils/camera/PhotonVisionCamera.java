package frc.robot.utils.camera;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.PoseSensorFusion;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
import frc.robot.utils.camera.VisionCameraEstimate.TXTYMeasurement;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera implements IVisionCamera {

    private static final ImmutableList<RawVisionFiducial> ALL_SIM_TAGS =
            ImmutableList.copyOf(Constants.Game.APRILTAG_LAYOUT.getTags().stream()
                    .map(v -> new RawVisionFiducial(v.ID, 0.1, 6, 7, 0))
                    .toList());

    private static final double DEFAULT_CLOSE_MAX_DISTANCE = Units.feetToMeters(7);
    private static final double DEFAULT_MAX_POSE_ERROR = 10; // 10 meters
    private static final double DEFAULT_TXTY_MAX_DISTANCE = 1.0; // have to be 1.0 meters or closer to use txty

    private static final double ROTATION_STDDEV_MULTIPLIER = 5.0;

    private static final double MAPLE_SIM_STDDEV = 0.001;
    private static final double MAPLE_SIM_TAG_DIST = 6;
    private static final double MAPLE_SIM_TAG_AREA = 0.1;

    private int numTags = 0;
    private boolean hasVision = false;
    private boolean connected = false;

    private double currentMeasurementStdDevs =
            PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS; // large number means less confident
    private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
    private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

    private String name;

    private final double stdMultiplier;
    private final double closeMaxDistance;
    private final double txtyMaxDistance;

    private final double maxPoseError;

    private CameraType physicalCamera;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimatorClose;
    private final PhotonPoseEstimator photonEstimatorFar;
    private PhotonPoseEstimator photonEstimatorTXTY;

    private Pose2d txtyPose = new Pose2d();

    private final Alert disconnectedAlert;

    public PhotonVisionCamera(String name, CameraType physicalCamera, Transform3d robotToCamera, double stdMultiplier) {
        this.name = name;
        this.physicalCamera = physicalCamera;
        this.stdMultiplier = stdMultiplier;
        this.closeMaxDistance = DEFAULT_CLOSE_MAX_DISTANCE;
        this.txtyMaxDistance = DEFAULT_TXTY_MAX_DISTANCE;
        this.maxPoseError = DEFAULT_MAX_POSE_ERROR;

        camera = new PhotonCamera(name);

        photonEstimatorClose = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonEstimatorClose.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorFar = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
        photonEstimatorFar.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorTXTY = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera);

        this.disconnectedAlert = new Alert("PhotonCamera " + name + " disconnected!", Alert.AlertType.kError);
        disconnectedAlert.set(true);

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
                && Constants.RobotState.VISION_SIMULATION_MODE.isPhotonSim()) {
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(
                    physicalCamera.getDetectorWidth(),
                    physicalCamera.getDetectorHeight(),
                    Rotation2d.fromDegrees(physicalCamera.fov));
            cameraProp.setCalibError(physicalCamera.pxError, physicalCamera.pxErrorStdDev);
            cameraProp.setFPS(physicalCamera.fps);
            cameraProp.setAvgLatencyMs(physicalCamera.latencyMs);
            cameraProp.setLatencyStdDevMs(physicalCamera.latencyStdDevMs);

            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableDrawWireframe(true);
            RobotContainer.visionSim.addCamera(cameraSim, robotToCamera);
        }
    }

    @Override
    public boolean isConnected() {
        return connected;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public CameraType getCameraType() {
        return physicalCamera;
    }

    @Override
    public boolean hasVision() {
        return hasVision;
    }

    @Override
    public int getNumTags() {
        return numTags;
    }

    @Override
    public VisionCameraEstimate getCurrentEstimate() {
        return currentEstimate;
    }

    @Override
    public VisionCameraEstimate getUnsafeEstimate() {
        return unsafeEstimate;
    }

    @Override
    public double getMeasurementStdDevs() {
        return currentMeasurementStdDevs;
    }

    @Override
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void updateEstimation() {
        updateConnectionStatus();
        if (!connected) {
            resetVisionState();
            return;
        }

        Measurements measurements = getMeasurements();

        if (measurements.isEmpty()) {
            resetVisionState();
            return;
        }

        Optional<TXTYMeasurement> txtyMeasurement = measurements.txty();

        if (txtyMeasurement.isEmpty()) {
            txtyPose = new Pose2d();
        } else {
            txtyPose = txtyMeasurement.get().pose();
        }

        VisionCameraEstimate selectedMeasurement = selectBestMeasurement(measurements);
        validateVisionMeasurement(selectedMeasurement);
        hasVision = false; // will be set true if not ignored when adding to fusion
    }

    private Measurements getMeasurements() {
        if (isRealOrPhotonSim()) {
            return getPhotonMeasurements();
        }
        return getMapleSimMeasurements();
    }

    private static boolean isRealOrPhotonSim() {
        return Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                || Constants.RobotState.VISION_SIMULATION_MODE.isPhotonSim();
    }

    private Measurements getPhotonMeasurements() {
        addHeadingDataToEstimators();
        return getPhotonVisionResults();
    }

    private void addHeadingDataToEstimators() {
        double timestamp = Timer.getTimestamp();
        Rotation3d heading = new Rotation3d(
                0,
                0,
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians());

        photonEstimatorClose.addHeadingData(timestamp, heading);
        photonEstimatorFar.addHeadingData(timestamp, heading);
        photonEstimatorTXTY.addHeadingData(timestamp, heading);
    }

    private Measurements getMapleSimMeasurements() {
        Pose2d maplePose = RobotContainer.model.getRobot();
        if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.MAPLE_NOISE) {
            maplePose = SimpleMath.poseNoise(maplePose, MAPLE_SIM_STDDEV, MAPLE_SIM_STDDEV);
        }

        VisionCameraEstimate closeEstimate = new VisionCameraEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(physicalCamera.latencyMs),
                ALL_SIM_TAGS.size(),
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                false,
                false,
                -1);
        VisionCameraEstimate farEstimate = new VisionCameraEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(physicalCamera.latencyMs),
                ALL_SIM_TAGS.size(),
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                true,
                false,
                -1);

        return new Measurements(
                closeEstimate,
                farEstimate,
                Optional.of(new TXTYMeasurement(maplePose, Timer.getTimestamp(), MAPLE_SIM_TAG_DIST, -1)));
    }

    private void updateConnectionStatus() {
        connected = camera.isConnected();
    }

    private void resetVisionState() {
        hasVision = false;
        numTags = 0;
        currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
    }

    private VisionCameraEstimate selectBestMeasurement(Measurements measurements) {
        VisionCameraEstimate measurement = calculateConfidenceAndEstimate(measurements);
        numTags = measurement.tagCount();
        unsafeEstimate = measurement;

        return measurement;
    }

    private VisionCameraEstimate calculateConfidenceAndEstimate(Measurements measurements) {
        VisionCameraEstimate closeEst = measurements.getCloseMeasurement();

        if (DashboardUI.Autonomous.isForceMT1Pressed()) {
            currentMeasurementStdDevs = physicalCamera.calculateStdDevs(closeEst.avgTagDist()) * stdMultiplier;
            return closeEst;
        }

        if (closeEst.tagCount() > 0) {
            Optional<TXTYMeasurement> txtyMeasurement = measurements.txty();

            if (txtyMeasurement.isPresent()
                    && SimpleMath.isInField(txtyMeasurement.get().pose())
                    && txtyMeasurement.get().distToCamera() <= txtyMaxDistance) {
                currentMeasurementStdDevs = physicalCamera.txtyStdDevs * stdMultiplier;
                return new VisionCameraEstimate(txtyMeasurement.get());
            }

            VisionCameraEstimate farEst = measurements.getFarMeasurement();

            if (SimpleMath.isInField(closeEst.pose()) && closeEst.avgTagDist() < closeMaxDistance) {
                currentMeasurementStdDevs = physicalCamera.calculateStdDevs(closeEst.avgTagDist()) * stdMultiplier;
                return closeEst;
            } else if (SimpleMath.isInField(farEst.pose())) {
                currentMeasurementStdDevs = physicalCamera.calculateStdDevs(farEst.avgTagDist()) * stdMultiplier;
                return farEst;
            }
        }

        currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        return closeEst;
    }

    private void validateVisionMeasurement(VisionCameraEstimate measurement) {
        if (currentMeasurementStdDevs >= PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS
                || isPoseErrorTooLarge(measurement)) {
            hasVision = false;
            numTags = 0;
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }
    }

    private boolean isPoseErrorTooLarge(VisionCameraEstimate measurement) {
        return measurement
                        .pose()
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > maxPoseError;
    }

    @Override
    public void addVisionMeasurement(boolean trust, Pose2d closestPose) {
        if (currentMeasurementStdDevs < PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS) {
            hasVision = true;
            currentEstimate = unsafeEstimate;

            if (closestPose != null) {
                double distanceToClosest =
                        currentEstimate.pose().getTranslation().getDistance(closestPose.getTranslation());
                currentMeasurementStdDevs += distanceToClosest / 2.0;
            }

            RobotContainer.poseSensorFusion.addVisionMeasurement(
                    currentEstimate.pose(),
                    currentEstimate.timestampSeconds(),
                    VecBuilder.fill(
                            currentMeasurementStdDevs,
                            currentMeasurementStdDevs,
                            trust
                                    ? currentMeasurementStdDevs * ROTATION_STDDEV_MULTIPLIER
                                    : PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS));
        }
    }

    private record Measurements(VisionCameraEstimate close, VisionCameraEstimate far, Optional<TXTYMeasurement> txty) {
        private boolean isEmpty() {
            return close == null && far == null;
        }

        private VisionCameraEstimate getCloseMeasurement() {
            return close == null ? far : close;
        }

        private VisionCameraEstimate getFarMeasurement() {
            return far == null ? close : far;
        }
    }

    @Override
    public void logValues(String id) {
        String prefix = "PhotonCamera/" + id + "/";
        Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose());
        Logger.recordOutput(prefix + "AvgDist", unsafeEstimate.avgTagDist());
        Logger.recordOutput(prefix + "NumTags", numTags);
        Logger.recordOutput(prefix + "Confidence", currentMeasurementStdDevs);
        Logger.recordOutput(prefix + "HasVision", hasVision);
        Logger.recordOutput(prefix + "Connected", connected);
        Logger.recordOutput(prefix + "TXTY", txtyPose);

        disconnectedAlert.set(!connected);
    }

    @SuppressWarnings("java:S3553") // input is from output of another function
    private VisionCameraEstimate getVisionCameraEstimateFromPhotonEstimate(
            Optional<EstimatedRobotPose> estOpt, PhotonPoseEstimator estimator, boolean isConstrained) {
        if (estOpt.isPresent()) {
            EstimatedRobotPose est = estOpt.get();
            int targetNum = est.targetsUsed.size();

            double avgTagDist = 0;
            double avgTagArea = 0;

            ImmutableList.Builder<RawVisionFiducial> rawFiducials = ImmutableList.builderWithExpectedSize(targetNum);
            for (PhotonTrackedTarget target : est.targetsUsed) {
                double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                double distRobot = target.getBestCameraToTarget()
                        .plus(estimator.getRobotToCameraTransform())
                        .getTranslation()
                        .getNorm();
                avgTagDist += dist;
                avgTagArea += target.getArea();

                RawVisionFiducial rawFiducial = new RawVisionFiducial(
                        target.fiducialId, target.getArea(), dist, distRobot, target.getPoseAmbiguity());
                rawFiducials.add(rawFiducial);
            }
            avgTagDist /= targetNum;
            avgTagArea /= targetNum;

            return new VisionCameraEstimate(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    physicalCamera.latencyMs,
                    targetNum,
                    avgTagDist,
                    avgTagArea,
                    rawFiducials.build(),
                    isConstrained,
                    false,
                    -1);
        }

        return null;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Measurements getPhotonVisionResults() {
        Optional<EstimatedRobotPose> closeOpt = Optional.empty();
        Optional<EstimatedRobotPose> farOpt = Optional.empty();
        Optional<EstimatedRobotPose> txtyOpt = Optional.empty();

        double bestTargetDist = Double.MAX_VALUE;

        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            closeOpt = photonEstimatorClose.update(change);
            farOpt = photonEstimatorFar.update(change);
            txtyOpt = photonEstimatorTXTY.update(change);

            if (change.hasTargets()) {
                bestTargetDist = change.getBestTarget()
                        .getBestCameraToTarget()
                        .getTranslation()
                        .getNorm();
            }
        }

        final double bestTargetDistFinal = bestTargetDist;

        return new Measurements(
                getVisionCameraEstimateFromPhotonEstimate(closeOpt, photonEstimatorClose, false),
                getVisionCameraEstimateFromPhotonEstimate(farOpt, photonEstimatorFar, true),
                txtyOpt.map(v -> new TXTYMeasurement(
                        v.estimatedPose.toPose2d(),
                        v.timestampSeconds,
                        bestTargetDistFinal,
                        v.targetsUsed.get(0).fiducialId)));
    }
}
