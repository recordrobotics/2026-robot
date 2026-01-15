package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.SimpleMath;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class CoralDetection extends ManagedSubsystemBase {

    private static final double PITCH_EXTENT = 28.7; // degrees
    private static final double YAW_EXTENT = 27.4; // degrees
    private static final double TRUE_PITCH_EXTENT = 40; // degrees
    private static final double TRUE_YAW_EXTENT = 39.8; // degrees
    private static final double ASPECT_RATIO_MIN = 0.5;
    private static final double ASPECT_RATIO_MAX = 2.5;

    // no IO since simulation does not currently support non-apriltag pipelines, so no point
    private PhotonCamera camera;
    // instead simulation can choose between maple sim coral or external photonvision client
    private CoralDetectionSimulationMode simulationMode = Constants.RobotState.CORAL_DETECTION_SIMULATION_MODE;

    public enum CoralDetectionSimulationMode {
        /**
         * Use external photonvision camera connected to the same network as simulation
         */
        PHOTONVISION,
        /**
         * Use maple sim to "detect" coral
         */
        MAPLE_SIM
    }

    private final Alert cameraDisconnectedAlert;

    public static final class DetectedCoral {
        private Pose3d fieldPose;
        private double timestamp;

        private DetectedCoral(Pose3d fieldPose, double timestamp) {
            this.fieldPose = fieldPose;
            this.timestamp = timestamp;
        }

        public Pose3d getFieldPose() {
            return fieldPose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    record PhotonTrackedTargetTimestamped(PhotonTrackedTarget target, double timestamp) {}

    private List<DetectedCoral> detectedCorals = new ArrayList<>();

    public CoralDetection() {
        camera = new PhotonCamera(Constants.PhotonVision.PHOTON_CORAL_INTAKE);
        cameraDisconnectedAlert = new Alert("Coral Detection Camera Disconnected", Alert.AlertType.kError);
        cameraDisconnectedAlert.set(true);
    }

    @Override
    public void periodicManaged() {
        if (shouldProcessPhotonVisionTargets()) {
            processPhotonVisionTargets();
        } else {
            cameraDisconnectedAlert.set(false);
        }
    }

    private boolean shouldProcessPhotonVisionTargets() {
        return Constants.RobotState.getMode() == Mode.REAL
                || simulationMode == CoralDetectionSimulationMode.PHOTONVISION;
    }

    private void processPhotonVisionTargets() {
        List<PhotonTrackedTargetTimestamped> targets = collectTargetsFromCamera();
        updateDetectedCorals(targets);
        removeOldCorals();
    }

    private List<PhotonTrackedTargetTimestamped> collectTargetsFromCamera() {
        List<PhotonTrackedTargetTimestamped> targets = new ArrayList<>();

        checkCameraConnection();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                targets.addAll(result.getTargets().stream()
                        .filter(target -> target.objDetectId == Constants.PhotonVision.CORAL_ID)
                        .map(target -> new PhotonTrackedTargetTimestamped(target, result.getTimestampSeconds()))
                        .toList());
            }
        }

        return targets;
    }

    private void checkCameraConnection() {
        cameraDisconnectedAlert.set(!camera.isConnected());

        if (Constants.RobotState.getMode() == Mode.SIM && !camera.isConnected()) {
            ConsoleLogger.logError(
                    "Coral Detection camera not connected! Did you forget to setSimulationMode(CoralDetectionSimulationMode.MAPLE_SIM)?");
        }
    }

    private void updateDetectedCorals(List<PhotonTrackedTargetTimestamped> targets) {
        List<Transform3d> projected = project2dPoses(targets);
        for (int i = 0; i < projected.size(); i++) {
            Pose3d projectedField = calculateProjectedFieldPose(targets.get(i), projected.get(i));
            updateOrAddCoral(projectedField, targets.get(i).timestamp);
        }
    }

    private static Pose3d calculateProjectedFieldPose(PhotonTrackedTargetTimestamped target, Transform3d projection) {
        Pose3d robot3d = new Pose3d(RobotContainer.poseSensorFusion
                .getEstimatedPositionAt(target.timestamp)
                .orElseGet(() -> RobotContainer.poseSensorFusion.getEstimatedPosition()));
        Pose3d camera3d = robot3d.transformBy(Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE);
        return camera3d.transformBy(projection);
    }

    private void updateOrAddCoral(Pose3d projectedField, double timestamp) {
        DetectedCoral existingCoral = findExistingCoral(projectedField);

        if (existingCoral != null) {
            updateExistingCoral(existingCoral, projectedField, timestamp);
        } else {
            detectedCorals.add(new DetectedCoral(projectedField, timestamp));
        }
    }

    private DetectedCoral findExistingCoral(Pose3d projectedField) {
        for (DetectedCoral coral : detectedCorals) {
            if (projectedField.getTranslation().getDistance(coral.fieldPose.getTranslation())
                    < Constants.PhotonVision.CORAL_ID_DISTANCE.in(Meters)) {
                return coral;
            }
        }
        return null;
    }

    private static void updateExistingCoral(DetectedCoral coral, Pose3d projectedField, double timestamp) {
        if (timestamp > coral.timestamp) {
            coral.fieldPose = projectedField;
            coral.timestamp = timestamp;
        }
    }

    private void removeOldCorals() {
        detectedCorals.removeIf(coral -> {
            double age = Timer.getTimestamp() - coral.timestamp;
            return age > Constants.PhotonVision.CORAL_TIMEOUT.in(Seconds) || age < 0;
        });
    }

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] getCorals() {
        if (Constants.RobotState.getMode() == Mode.REAL
                || simulationMode == CoralDetectionSimulationMode.PHOTONVISION) {
            Pose3d[] corals = new Pose3d[detectedCorals.size()];
            for (int i = 0; i < detectedCorals.size(); i++) {
                DetectedCoral coral = detectedCorals.get(i);
                corals[i] = coral.fieldPose;
            }
            return corals;
        } else {
            return SimulatedArena.getInstance()
                    .getGamePiecesPosesByType("Coral")
                    .toArray(new Pose3d[0]);
        }
    }

    public void setSimulationMode(CoralDetectionSimulationMode mode) {
        simulationMode = mode;
    }

    public CoralDetectionSimulationMode getSimulationMode() {
        return simulationMode;
    }

    /**
     * Uses law of sines assuming a constant height ground plane to project 2d targets into 3d
     *
     * @param targets list of 2d targets from photonvision
     * @return list of transforms from camera to target in 3d space
     */
    private static List<Transform3d> project2dPoses(List<PhotonTrackedTargetTimestamped> targets) {
        return targets.stream()
                .map(t -> {
                    PhotonTrackedTarget target = t.target;
                    double theta = Math.PI / 2
                            + Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                    .getRotation()
                                    .getMeasureY()
                                    .in(Radians)
                            + Units.degreesToRadians(SimpleMath.remap(
                                    target.getPitch(),
                                    -PITCH_EXTENT,
                                    PITCH_EXTENT,
                                    -TRUE_PITCH_EXTENT,
                                    TRUE_PITCH_EXTENT));

                    double forward = Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                    .getTranslation()
                                    .getMeasureZ()
                                    .in(Meters)
                            * Math.tan(theta);

                    double minX = Double.MAX_VALUE;
                    double maxX = Double.MIN_VALUE;
                    double minY = Double.MAX_VALUE;
                    double maxY = Double.MIN_VALUE;

                    for (TargetCorner corner : target.minAreaRectCorners) {
                        minX = Math.min(minX, corner.x);
                        maxX = Math.max(maxX, corner.x);
                        minY = Math.min(minY, corner.y);
                        maxY = Math.max(maxY, corner.y);
                    }

                    double width = maxX - minX;
                    double height = maxY - minY;
                    double aspectRatio = width / height;

                    double cameraRelativeYaw =
                            Math.PI / 2 * MathUtil.inverseInterpolate(ASPECT_RATIO_MIN, ASPECT_RATIO_MAX, aspectRatio);

                    return new Transform3d(
                            new Translation3d(
                                            forward,
                                            0,
                                            -Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                                    .getTranslation()
                                                    .getMeasureZ()
                                                    .in(Meters))
                                    .rotateBy(new Rotation3d(
                                            0,
                                            -Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                                    .getRotation()
                                                    .getY(),
                                            Units.degreesToRadians(-SimpleMath.remap(
                                                    target.getYaw(),
                                                    -YAW_EXTENT,
                                                    YAW_EXTENT,
                                                    -TRUE_YAW_EXTENT,
                                                    TRUE_YAW_EXTENT)))),
                            new Rotation3d(
                                            0,
                                            0,
                                            cameraRelativeYaw
                                                    + Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                                            .getRotation()
                                                            .getZ())
                                    .rotateBy(Constants.PhotonVision.ROBOT_TO_CAMERA_GROUND_INTAKE
                                            .getRotation()
                                            .unaryMinus()));
                })
                .toList();
    }
}
