package frc.robot.utils.camera.poseestimation.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.poseestimation.CameraPoseEstimate;
import frc.robot.utils.camera.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import frc.robot.utils.camera.poseestimation.PoseEstimationCamera;
import frc.robot.utils.libraries.LimelightHelpers;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import java.util.List;
import java.util.Optional;

/**
 * Limelight protocol camera for pose estimation.
 */
public class LimelightCamera extends PoseEstimationCamera {

    private static final double TIMEOUT_FPS = 3.0;
    private static final double TIMEOUT_SECONDS = 1.0 / TIMEOUT_FPS;

    private double lastHeartbeatIncrementTime = 0.0;
    private double lastHeartbeatValue = -1.0;

    /**
     * The transform from robot to camera.
     */
    private Transform3d robotToCamera;

    /**
     * Constructs a LimelightCamera with the given name and physical camera type and default settings.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera.
     */
    public LimelightCamera(String name, PhysicalCamera physicalCamera, Transform3d robotToCamera) {
        super(name, physicalCamera);
        this.robotToCamera = robotToCamera;
    }

    /**
     * Constructs a LimelightCamera with the given name and physical camera type.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera.
     * @param maxDistanceToCurrentEstimate Maximum distance from last pose to accept vision measurements.
     * @param unconstrainedMaxDistance Maximum distance to tag to use unconstrained measurements.
     * @param txtyMaxDistance Maximum distance to TXTY tag to use TXTY measurements.
     * @param rotationStdDevMultiplier Multiplier for rotation standard deviation when using rotation measurements.
     */
    protected LimelightCamera(
            String name,
            PhysicalCamera physicalCamera,
            Transform3d robotToCamera,
            double maxDistanceToCurrentEstimate,
            double unconstrainedMaxDistance,
            double txtyMaxDistance,
            double rotationStdDevMultiplier) {
        super(
                name,
                physicalCamera,
                maxDistanceToCurrentEstimate,
                unconstrainedMaxDistance,
                txtyMaxDistance,
                rotationStdDevMultiplier);
        this.robotToCamera = robotToCamera;
    }

    /**
     * Checks if the camera is connected.
     * <p>This is different from being enabled. A camera can be enabled but not connected.
     * @return True if the camera is connected, false otherwise.
     */
    @Override
    public boolean isConnected() {
        double heartbeat = LimelightHelpers.getHeartbeat(getName());
        double currentTime = Timer.getTimestamp();

        if (heartbeat > lastHeartbeatValue) {
            lastHeartbeatIncrementTime = currentTime;
            lastHeartbeatValue = heartbeat;
            return true;
        } else {
            return currentTime - lastHeartbeatIncrementTime < TIMEOUT_SECONDS;
        }
    }

    /**
     * Sets the current pipeline index.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(getName(), pipeline);
    }

    /**
     * Gets the transform from robot to camera.
     * @return The transform from robot to camera.
     */
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    /**
     * Sets the transform from robot to camera.
     * @param robotToCamera The transform from robot to camera.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    @Override
    public List<CameraPoseEstimate> makeEstimates() {
        updateCameraOrientation();

        Measurements measurements = getMeasurements();

        if (!isConnected() || measurements.mt1() == null || measurements.mt1().tagCount == 0) {
            return List.of();
        }

        CameraPoseEstimate estimate = new CameraPoseEstimate(
                measurements.mt1().pose,
                Optional.ofNullable(measurements.mt2()).map(m -> m.pose),
                measurements.txty(),
                measurements.mt1().timestampSeconds,
                measurements.mt1().latency,
                measurements.mt1().tagCount,
                measurements.mt1().avgTagDist,
                measurements.mt1().avgTagArea);

        return List.of(estimate);
    }

    /**
     * Updates the camera orientation in the Limelight network tables.
     */
    private void updateCameraOrientation() {
        Rotation2d estimatedRotation =
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation();

        double yawRate = RobotContainer.poseSensorFusion.nav.getYawRate();

        LimelightHelpers.SetRobotOrientation(
                getName(),
                estimatedRotation.getDegrees(),
                0, // TODO: try out `yawRate`,
                0,
                0,
                0,
                0);

        LimelightHelpers.setCameraPose_RobotSpace(
                getName(),
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                robotToCamera.getRotation().getX(),
                robotToCamera.getRotation().getY(),
                robotToCamera.getRotation().getZ());
    }

    /**
     * Gets the measurements from the Limelight.
     * @return The measurements from the Limelight.
     */
    private Measurements getMeasurements() {
        PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(getName());
        PoseEstimate measurementM2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
        List<TXTYMeasurement> txtyMeasurement = measurement == null
                ? List.of()
                : pnpDistanceTrigSolveStrategy(measurement.timestampSeconds, measurement.tagCount);

        return new Measurements(measurement, measurementM2, txtyMeasurement);
    }

    /**
     * From PhotonVision
     * @param timestamp the timestamp of the measurement
     * @param tagCount number of tags seen
     * @return the TXTYMeasurements
     */
    @SuppressWarnings("java:S1941") // library code
    private List<TXTYMeasurement> pnpDistanceTrigSolveStrategy(double timestamp, int tagCount) {
        if (tagCount == 0) {
            return List.of();
        }

        Optional<Pose2d> headingSampleOpt = RobotContainer.poseSensorFusion.getEstimatedPositionAt(timestamp);
        if (headingSampleOpt.isEmpty()) {
            return List.of();
        }
        Rotation2d headingSample = headingSampleOpt.get().getRotation();

        Pose3d bestCameraToTarget = LimelightHelpers.toPose3D(LimelightHelpers.getTargetPose_CameraSpace(getName()));

        Translation2d camToTagTranslation = new Translation3d(
                        bestCameraToTarget.getTranslation().getNorm(),
                        new Rotation3d(
                                0,
                                -Math.toRadians(LimelightHelpers.getTYNC(getName())),
                                -Math.toRadians(LimelightHelpers.getTXNC(getName()))))
                .rotateBy(robotToCamera.getRotation())
                .toTranslation2d()
                .rotateBy(headingSample);

        int fiducialId = (int) LimelightHelpers.getFiducialID(getName());

        Optional<Pose3d> tagPoseOpt = Constants.Game.APRILTAG_LAYOUT.getTagPose(fiducialId);
        if (tagPoseOpt.isEmpty()) {
            return List.of();
        }
        Pose2d tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation = tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
                robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(headingSample);

        Pose2d robotPose = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);

        return List.of(new TXTYMeasurement(
                robotPose, bestCameraToTarget.getTranslation().getNorm(), fiducialId));
    }

    /**
     * A record representing the measurements from the Limelight.
     * @param mt1 The first pose estimate measurement.
     * @param mt2 The second pose estimate measurement.
     * @param txty The list of TXTY measurements.
     */
    private record Measurements(PoseEstimate mt1, PoseEstimate mt2, List<TXTYMeasurement> txty) {}
}
