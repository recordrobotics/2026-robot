package frc.robot.utils.camera.poseestimation.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.poseestimation.CameraPoseEstimate;
import frc.robot.utils.camera.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import frc.robot.utils.camera.poseestimation.PoseEstimationCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision protocol camera for pose estimation.
 */
public class PhotonVisionCamera extends PoseEstimationCamera {

    /**
     * The transform from robot to camera.
     */
    private Transform3d robotToCamera;

    /**
     * The PhotonCamera instance.
     */
    private final PhotonCamera camera;

    /**
     * The PhotonPoseEstimator instance.
     */
    private final PhotonPoseEstimator photonEstimator;

    /**
     * Constructs a PhotonVisionCamera with the given name, physical camera, and robot-to-camera transform.
     * @param name The name of the camera.
     * @param physicalCamera The physical camera type.
     * @param robotToCamera The transform from robot to camera.
     */
    public PhotonVisionCamera(String name, PhysicalCamera physicalCamera, Transform3d robotToCamera) {
        super(name, physicalCamera);

        camera = new PhotonCamera(name);

        this.robotToCamera = robotToCamera;
        photonEstimator = new PhotonPoseEstimator(Constants.Game.APRILTAG_LAYOUT, robotToCamera);

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
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

    /**
     * Checks if the camera is connected.
     * <p>This is different from being enabled. A camera can be enabled but not connected.
     * @return True if the camera is connected, false otherwise.
     */
    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    /**
     * Sets the pipeline index for the camera.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    /**
     * Sets the transform from robot to camera.
     * @param robotToCamera The transform from robot to camera.
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Gets the transform from robot to camera.
     * @return The transform from robot to camera.
     */
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    @Override
    public List<CameraPoseEstimate> makeEstimates() {
        if (!isConnected()) {
            return List.of();
        }

        addHeadingDataToEstimator();
        return getPhotonVisionResults();
    }

    /**
     * Adds the current robot heading data to the PhotonPoseEstimator.
     */
    private void addHeadingDataToEstimator() {
        double timestamp = Timer.getTimestamp();
        Rotation3d heading = new Rotation3d(
                0,
                0,
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians());

        photonEstimator.addHeadingData(timestamp, heading);
        photonEstimator.setRobotToCameraTransform(robotToCamera);
    }

    /**
     * Gets the pose estimates from PhotonVision results.
     * @return A list of camera pose estimates.
     */
    private List<CameraPoseEstimate> getPhotonVisionResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return List.of();
        }

        List<CameraPoseEstimate> measurements = new ArrayList<>();

        Optional<Matrix<N3, N3>> cameraMatrix = camera.getCameraMatrix();
        Optional<Matrix<N8, N1>> distortionCoeffs = camera.getDistCoeffs();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            Optional<EstimatedRobotPose> unconstrainedEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (unconstrainedEst.isEmpty()) {
                unconstrainedEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            Optional<EstimatedRobotPose> constrainedEst;
            if (unconstrainedEst.isPresent() && cameraMatrix.isPresent() && distortionCoeffs.isPresent()) {
                constrainedEst = photonEstimator.estimateConstrainedSolvepnpPose(
                        result,
                        cameraMatrix.get(),
                        distortionCoeffs.get(),
                        unconstrainedEst.get().estimatedPose,
                        false,
                        1.0);
            } else {
                constrainedEst = Optional.empty();
            }

            Optional<EstimatedRobotPose> txtyEst = photonEstimator.estimatePnpDistanceTrigSolvePose(result);
            double bestTargetDist = result.getBestTarget()
                    .getBestCameraToTarget()
                    .getTranslation()
                    .getNorm();

            if (unconstrainedEst.isPresent()
                    && !unconstrainedEst.get().targetsUsed.isEmpty()) {
                measurements.add(getCameraPoseEstimateFromPhotonEstimates(
                        unconstrainedEst.get(),
                        constrainedEst,
                        txtyEst.map(v -> new TXTYMeasurement(
                                v.estimatedPose.toPose2d(), bestTargetDist, v.targetsUsed.get(0).fiducialId))));
            }
        }

        return measurements;
    }

    /**
     * Converts PhotonVision estimated robot poses to a CameraPoseEstimate.
     * @param unconstrainedEst The unconstrained estimated robot pose.
     * @param constrainedEst The constrained estimated robot pose.
     * @param txtyEst The TXTY estimated robot pose.
     * @return A CameraPoseEstimate object representing the combined pose estimates.
     */
    @SuppressWarnings("java:S3553") // input is from output of another function
    private CameraPoseEstimate getCameraPoseEstimateFromPhotonEstimates(
            EstimatedRobotPose unconstrainedEst,
            Optional<EstimatedRobotPose> constrainedEst,
            Optional<TXTYMeasurement> txtyEst) {

        int tagCount = unconstrainedEst.targetsUsed.size();

        double avgTagDist = 0;
        double avgTagArea = 0;

        for (PhotonTrackedTarget target : unconstrainedEst.targetsUsed) {
            double dist = target.getBestCameraToTarget().getTranslation().getNorm();
            avgTagDist += dist;
            avgTagArea += target.getArea();
        }
        avgTagDist /= tagCount;
        avgTagArea /= tagCount;

        return new CameraPoseEstimate(
                unconstrainedEst.estimatedPose.toPose2d(),
                constrainedEst.map(e -> e.estimatedPose.toPose2d()),
                txtyEst.map(List::of).orElse(List.of()),
                unconstrainedEst.timestampSeconds,
                getPhysicalCamera().latencyMs,
                tagCount,
                avgTagDist,
                avgTagArea);
    }
}
