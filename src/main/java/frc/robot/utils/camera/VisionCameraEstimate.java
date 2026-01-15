package frc.robot.utils.camera;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import frc.robot.utils.libraries.LimelightHelpers.RawFiducial;

public record VisionCameraEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double avgTagDist,
        double avgTagArea,
        ImmutableList<RawVisionFiducial> rawFiducials,
        boolean isConstrained,
        boolean isTXTY,
        int txtyId) {

    public record RawVisionFiducial(int id, double area, double distToCamera, double distToRobot, double ambiguity) {
        public RawVisionFiducial(RawFiducial limelightFiducial) {
            this(
                    limelightFiducial.id,
                    limelightFiducial.ta,
                    limelightFiducial.distToCamera,
                    limelightFiducial.distToRobot,
                    limelightFiducial.ambiguity);
        }
    }

    @SuppressWarnings("java:S4738")
    public VisionCameraEstimate() {
        this(new Pose2d(), 0, 0, 0, 0, 0, ImmutableList.of(), false, false, -1);
    }

    public VisionCameraEstimate(PoseEstimate limelightEstimate) {
        this(
                limelightEstimate.pose,
                limelightEstimate.timestampSeconds,
                limelightEstimate.latency,
                limelightEstimate.tagCount,
                limelightEstimate.avgTagDist,
                limelightEstimate.avgTagArea,
                convertRawFiducials(limelightEstimate.rawFiducials),
                limelightEstimate.isMegaTag2,
                false,
                -1);
    }

    public record TXTYMeasurement(Pose2d pose, double timestamp, double distToCamera, int tagId) {}

    @SuppressWarnings("java:S4738") // we want specifically ImmutableList.of() here
    public VisionCameraEstimate(TXTYMeasurement txTyMeasurement) {
        this(
                txTyMeasurement.pose(),
                txTyMeasurement.timestamp(),
                0,
                1,
                txTyMeasurement.distToCamera(),
                0,
                ImmutableList.of(),
                false,
                true,
                txTyMeasurement.tagId);
    }

    private static ImmutableList<RawVisionFiducial> convertRawFiducials(RawFiducial[] rawFiducials) {
        ImmutableList.Builder<RawVisionFiducial> converted = ImmutableList.builderWithExpectedSize(rawFiducials.length);
        for (int i = 0; i < rawFiducials.length; i++) {
            converted.add(new RawVisionFiducial(rawFiducials[i]));
        }
        return converted.build();
    }
}
