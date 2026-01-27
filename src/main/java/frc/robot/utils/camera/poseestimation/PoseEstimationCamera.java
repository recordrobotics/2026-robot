package frc.robot.utils.camera.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PoseSensorFusion;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.poseestimation.CameraPoseEstimate.TXTYMeasurement;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract class for cameras that provide pose estimates.
 * <p>Supports both constrained and unconstrained pose estimates as well as a way to force using unconstrained estimates.
 * <p>Allows rotation measurements to be optionally trusted or ignored.
 */
public abstract class PoseEstimationCamera extends GenericCamera {

    public static final double DEFAULT_MAX_DISTANCE_TO_CURRENT_ESTIMATE = 10.0;
    public static final double DEFAULT_UNCONSTRAINED_MAX_DISTANCE = Units.feetToMeters(7);
    public static final double DEFAULT_TXTY_MAX_DISTANCE = 1.0;
    public static final double DEFAULT_ROTATION_STDDEV_MULTIPLIER = 5.0;

    /**
     * The SmartDashboard entry for whether to trust rotation measurements.
     */
    private static final String USE_ROTATION_ENTRY = "UseRotation";
    /**
     * The SmartDashboard entry for forcing unconstrained measurements.
     * <p>If true, unconstrained measurements will always be used regardless of distance.
     * <p>Useful for setting initial rotation on the field.
     */
    private static final String FORCE_UNCONSTRAINED_ENTRY = "ForceUnconstrained";

    /**
     * List of estimates from the camera from last periodic.
     */
    private List<CameraPoseEstimate> cachedEstimates = List.of();

    /**
     * Last used standard deviation for measurements.
     * Used for logging.
     */
    private double lastStdDev = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;

    /**
     * Whether to trust rotation measurements.
     */
    private boolean useRotation = true;

    /**
     * Whether to force using unconstrained measurements.
     */
    private boolean forceUnconstrained = false;

    /**
     * Maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     */
    private double maxDistanceToCurrentEstimate;

    /**
     * Maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     */
    private double unconstrainedMaxDistance;

    /**
     * Maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     */
    private double txtyMaxDistance;

    /**
     * Multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     */
    private double rotationStdDevMultiplier;

    /**
     * Constructs a PoseEstimationCamera with the given name and physical camera type and default settings.
     * <p>Sets up SmartDashboard entries for configuring rotation trust and forcing unconstrained measurements.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     */
    protected PoseEstimationCamera(String name, PhysicalCamera physicalCamera) {
        this(
                name,
                physicalCamera,
                DEFAULT_MAX_DISTANCE_TO_CURRENT_ESTIMATE,
                DEFAULT_UNCONSTRAINED_MAX_DISTANCE,
                DEFAULT_TXTY_MAX_DISTANCE,
                DEFAULT_ROTATION_STDDEV_MULTIPLIER);
    }

    /**
     * Constructs a PoseEstimationCamera with the given name and physical camera type.
     * <p>Sets up SmartDashboard entries for configuring rotation trust and forcing unconstrained measurements.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     * @param maxDistanceToCurrentEstimate Maximum distance from last pose to accept vision measurements.
     * @param unconstrainedMaxDistance Maximum distance to tag to use unconstrained measurements.
     * @param txtyMaxDistance Maximum distance to TXTY tag to use TXTY measurements.
     * @param rotationStdDevMultiplier Multiplier for rotation standard deviation when using rotation measurements.
     */
    protected PoseEstimationCamera(
            String name,
            PhysicalCamera physicalCamera,
            double maxDistanceToCurrentEstimate,
            double unconstrainedMaxDistance,
            double txtyMaxDistance,
            double rotationStdDevMultiplier) {
        super(name, physicalCamera);

        this.maxDistanceToCurrentEstimate = maxDistanceToCurrentEstimate;
        this.unconstrainedMaxDistance = unconstrainedMaxDistance;
        this.txtyMaxDistance = txtyMaxDistance;
        this.rotationStdDevMultiplier = rotationStdDevMultiplier;

        // Allow configuration from SmartDashboard
        SmartDashboard.putBoolean(getPrefix() + USE_ROTATION_ENTRY, useRotation);
        SmartDashboard.putBoolean(getPrefix() + FORCE_UNCONSTRAINED_ENTRY, forceUnconstrained);
    }

    /**
     * Whether rotation measurements are being trusted.
     * @return True if rotation measurements are trusted, false otherwise.
     */
    public boolean isUsingRotation() {
        return useRotation;
    }

    /**
     * Whether unconstrained measurements are being forced.
     * @return True if unconstrained measurements are forced, false otherwise.
     */
    public boolean isForcingUnconstrained() {
        return forceUnconstrained;
    }

    /**
     * Sets whether to trust rotation measurements.
     * @param useRotation True to trust rotation measurements, false to ignore them.
     */
    public void setUseRotation(boolean useRotation) {
        this.useRotation = useRotation;
        SmartDashboard.putBoolean(getPrefix() + USE_ROTATION_ENTRY, useRotation);
    }

    /**
     * Sets whether to force using unconstrained measurements.
     * @param forceUnconstrained True to force using unconstrained measurements, false to use normal logic.
     */
    public void setForceUnconstrained(boolean forceUnconstrained) {
        this.forceUnconstrained = forceUnconstrained;
        SmartDashboard.putBoolean(getPrefix() + FORCE_UNCONSTRAINED_ENTRY, forceUnconstrained);
    }

    /**
     * Whether the camera has valid vision estimates.
     * @return True if there are valid estimates, false otherwise.
     */
    public boolean hasVision() {
        return !cachedEstimates.isEmpty();
    }

    /**
     * Gets the number of tags detected in the latest estimate.
     * @return The number of tags detected, or 0 if no vision.
     */
    public int getNumTags() {
        if (!hasVision()) return 0;
        return cachedEstimates.get(cachedEstimates.size() - 1).tagCount();
    }

    /**
     * Gets the last standard deviation used in vision measurements.
     * @return The last standard deviation.
     */
    public double getLastStdDev() {
        return lastStdDev;
    }

    /**
     * Gets the maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     * @return The maximum distance to accept vision measurements.
     */
    public double getMaxDistanceToCurrentEstimate() {
        return maxDistanceToCurrentEstimate;
    }

    /**
     * Gets the maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     * @return The maximum distance to tag for unconstrained measurements.
     */
    public double getUnconstrainedMaxDistance() {
        return unconstrainedMaxDistance;
    }

    /**
     * Gets the maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     * @return The maximum distance to TXTY tag for TXTY measurements.
     */
    public double getTxtyMaxDistance() {
        return txtyMaxDistance;
    }

    /**
     * Gets the multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     * @return The rotation standard deviation multiplier.
     */
    public double getRotationStdDevMultiplier() {
        return rotationStdDevMultiplier;
    }

    /**
     * Sets the maximum distance from last pose to accept vision measurements.
     * <p>Prevents large jumps in pose due to bad vision measurements.
     * @param maxDistanceToCurrentEstimate The maximum distance to accept vision measurements.
     */
    public void setMaxDistanceToCurrentEstimate(double maxDistanceToCurrentEstimate) {
        this.maxDistanceToCurrentEstimate = maxDistanceToCurrentEstimate;
    }

    /**
     * Sets the maximum distance to tag to use unconstrained measurements.
     * <p>Beyond this distance, constrained measurements will be used if available.
     * @param unconstrainedMaxDistance The maximum distance to tag for unconstrained measurements.
     */
    public void setUnconstrainedMaxDistance(double unconstrainedMaxDistance) {
        this.unconstrainedMaxDistance = unconstrainedMaxDistance;
    }

    /**
     * Sets the maximum distance to TXTY tag to use TXTY measurements.
     * <p>Beyond this distance, TXTY measurements will be ignored.
     * @param txtyMaxDistance The maximum distance to TXTY tag for TXTY measurements.
     */
    public void setTxtyMaxDistance(double txtyMaxDistance) {
        this.txtyMaxDistance = txtyMaxDistance;
    }

    /**
     * Sets the multiplier for rotation standard deviation when using rotation measurements.
     * <p>Higher values mean less trust in rotation measurements.
     * @param rotationStdDevMultiplier The rotation standard deviation multiplier.
     */
    public void setRotationStdDevMultiplier(double rotationStdDevMultiplier) {
        this.rotationStdDevMultiplier = rotationStdDevMultiplier;
    }

    /**
     * Makes pose estimates from the camera.
     * @return A list of camera pose estimates.
     */
    protected abstract List<CameraPoseEstimate> makeEstimates();

    /**
     * Periodically updates the cached pose estimates from the camera.
     * <p>This method should be called regularly (e.g. in a main periodic loop).
     */
    @Override
    public void periodic() {
        if (!isEnabled()) {
            cachedEstimates = List.of();
            return;
        }

        cachedEstimates = makeEstimates();
    }

    /***
     * Adds vision measurements to the sensor fusion.
     * @param fusion The pose sensor fusion to add measurements to.
     * @param currentEstimate The current estimated pose from the fusion.
     * @param txtyId The ID of the TXTY measurement target.
     */
    public void addVisionMeasurements(PoseSensorFusion fusion, Pose2d currentEstimate, int txtyId) {
        for (CameraPoseEstimate estimate : cachedEstimates) {
            double stdDev = calculateStdDevs(fusion, estimate);

            Optional<Pose2d> pose;

            if (forceUnconstrained) {
                pose = Optional.of(estimate.unconstrainedPose());
            } else {
                Optional<TXTYMeasurement> txtyMeasurement = findValidTXTY(estimate, txtyId);

                if (txtyMeasurement.isPresent()) {
                    pose = Optional.of(txtyMeasurement.get().pose());
                    stdDev = getPhysicalCamera().txtyStdDevs;
                } else {
                    pose = choosePoseEstimate(estimate);
                }
            }

            if (pose.isPresent()
                    && SimpleMath.isInField(pose.get())
                    && pose.get().getTranslation().getDistance(currentEstimate.getTranslation())
                            <= maxDistanceToCurrentEstimate) {
                lastStdDev = stdDev;
                fusion.addVisionMeasurement(
                        pose.get(),
                        estimate.timestampSeconds(),
                        VecBuilder.fill(
                                stdDev,
                                stdDev,
                                useRotation
                                        ? stdDev * rotationStdDevMultiplier
                                        : PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS));
            }
        }
    }

    /**
     * Calculates the standard deviations for a given estimate.
     * @param fusion The pose sensor fusion.
     * @param estimate The camera pose estimate.
     * @return The calculated standard deviations.
     */
    private double calculateStdDevs(PoseSensorFusion fusion, CameraPoseEstimate estimate) {
        double stdDev = getPhysicalCamera().calculateStdDevs(estimate.avgTagDist());

        Optional<Pose2d> closestPose = fusion.getEstimatedPositionAt(estimate.timestampSeconds());
        if (closestPose.isPresent()) {
            double distanceToClosest = estimate.unconstrainedPose()
                    .getTranslation()
                    .getDistance(closestPose.get().getTranslation());
            stdDev += distanceToClosest / 2.0;
        }

        return stdDev;
    }

    /**
     * Finds a valid TXTY measurement within the maximum distance.
     * @param estimate The camera pose estimate.
     * @param txtyId The ID of the TXTY measurement target.
     * @return An optional TXTY measurement if found and valid.
     */
    private Optional<TXTYMeasurement> findValidTXTY(CameraPoseEstimate estimate, int txtyId) {
        for (TXTYMeasurement m : estimate.txtyMeasurements()) {
            if (m.tagId() == txtyId && m.distToCamera() <= txtyMaxDistance) {
                return Optional.of(m);
            }
        }

        return Optional.empty();
    }

    /**
     * Chooses between unconstrained and constrained pose estimates based on average tag distance.
     * @param estimate The camera pose estimate.
     * @return An optional chosen pose estimate.
     */
    private Optional<Pose2d> choosePoseEstimate(CameraPoseEstimate estimate) {
        if (estimate.avgTagDist() <= unconstrainedMaxDistance) {
            return Optional.of(estimate.unconstrainedPose());
        } else {
            return estimate.constrainedPose().or(() -> Optional.of(estimate.unconstrainedPose()));
        }
    }

    /**
     * Logs camera values.
     * <ul>
     * <li>[UnconstrainedPose]: Unconstrained Pose</li>
     * <li>[ConstrainedPose]: Constrained Pose</li>
     * <li>[AvgDist]: Average Tag Distance</li>
     * <li>[NumTags]: Number of Tags</li>
     * <li>[StdDev]: Standard Deviation</li>
     * <li>[TXTY]: TXTY Measurements</li>
     * <li>[HasVision]: Has Vision</li>
     * </ul>
     */
    @Override
    public void logValues() {
        super.logValues();

        String prefix = getPrefix();

        if (!cachedEstimates.isEmpty()) {
            CameraPoseEstimate lastEstimate = cachedEstimates.get(cachedEstimates.size() - 1);
            Logger.recordOutput(prefix + "UnconstrainedPose", lastEstimate.unconstrainedPose());
            lastEstimate.constrainedPose().ifPresent(p -> Logger.recordOutput(prefix + "ConstrainedPose", p));
            Logger.recordOutput(prefix + "AvgDist", lastEstimate.avgTagDist());
            Logger.recordOutput(prefix + "NumTags", lastEstimate.tagCount());
            Logger.recordOutput(prefix + "StdDev", lastStdDev);
            Logger.recordOutput(
                    prefix + "TXTY",
                    lastEstimate.txtyMeasurements().stream().map(v -> v.pose()).toArray(Pose2d[]::new));
        }

        Logger.recordOutput(prefix + "HasVision", hasVision());

        // Update from SmartDashboard
        useRotation = SmartDashboard.getBoolean(prefix + USE_ROTATION_ENTRY, true);
        forceUnconstrained = SmartDashboard.getBoolean(prefix + FORCE_UNCONSTRAINED_ENTRY, false);
    }
}
