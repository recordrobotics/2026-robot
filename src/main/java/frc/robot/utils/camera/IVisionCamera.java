package frc.robot.utils.camera;

import edu.wpi.first.math.geometry.Pose2d;

public interface IVisionCamera {

    String getName();

    CameraType getCameraType();

    boolean hasVision();

    boolean isConnected();

    double getMeasurementStdDevs();

    int getNumTags();

    VisionCameraEstimate getCurrentEstimate();

    VisionCameraEstimate getUnsafeEstimate();

    void setPipeline(int pipeline);

    void updateEstimation();

    /***
     * Add a vision measurement to the filter with weight scaled by distance to the closest pose.
     * @param trust whether to trust the rotation
     * @param closestPose the closest pose detected by the cameras, can be null
     */
    void addVisionMeasurement(boolean trust, Pose2d closestPose);

    void logValues(String id);
}
