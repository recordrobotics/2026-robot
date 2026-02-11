package frc.robot.utils.camera;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Enum representing different physical camera models with their specifications.
 */
public enum PhysicalCamera {
    /**
     * Limelight 2 Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-2
     */
    LIMELIGHT_2(1280, 960, 2, 74.36, 13, 0.2, 0.0005, 35, 5, 0.55, 1.4175, 0.35),

    /**
     * Limelight 3G Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3g
     */
    LIMELIGHT_3G(1280, 960, 1, 75.8, 8, 0.2, 0.0005, 35, 5, 0.55, 1.11574, 0.35),

    /**
     * Limelight 4 Camera Specifications
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4
     */
    LIMELIGHT_4(1280, 960, 1, 91.5, 30, 0.2, 0.0005, 35, 5, 0.55, 1.11574, 0.35),

    /**
     * SVPro Global Shutter Camera Specifications
     * https://www.amazon.com/SVPRO-Shutter-Distortion-Free-1920x1200-Computer/dp/B0CC28R5TL
     */
    SVPRO_GLOBAL_SHUTTER(1920, 1200, 1, 83, 23, 0.2, 0.0005, 35, 5, 0.55, 1.3, 0.35),

    /**
     * Arducam 100 FOV Camera Specifications
     * https://www.amazon.com/Arducam-Camera-Computer-Microphone-Windows/dp/B07ZRJDTBQ/?th=1
     */
    ARDUCAM_100_FOV(1920, 1080, 1, 100, 30, 0.2, 0.0005, 35, 5, 0.55, 1.0, 0.35),

    /**
     * Arducam 160 FOV Camera Specifications
     * https://www.amazon.com/Arducam-Camera-Computer-Microphone-Windows/dp/B07ZS75KZR/?th=1
     */
    ARDUCAM_160_FOV(1920, 1080, 1, 160, 30, 0.2, 0.0005, 35, 5, 0.55, 1.0, 0.35);

    public final int width;
    public final int height;
    public final double downscale;
    public final double fov; // diagonal
    public final double fps;
    public final double pxError;
    public final double pxErrorStdDev;
    public final double latencyMs;
    public final double latencyStdDevMs;

    public final double minStdDevs;
    public final double stdDevsExponent;
    public final double txtyStdDevs;

    public final double aspectRatio;
    public final double horizontalFov;
    public final double verticalFov;

    PhysicalCamera(
            int width,
            int height,
            double downscale,
            double fov, // diagonal
            double fps,
            double pxError,
            double pxErrorStdDev,
            double latencyMs,
            double latencyStdDevMs,
            double minStdDevs,
            double stdDevsExponent,
            double txtyStdDevs) {
        this.width = width;
        this.height = height;
        this.downscale = downscale;
        this.fov = fov;
        this.fps = fps;
        this.pxError = pxError;
        this.pxErrorStdDev = pxErrorStdDev;
        this.latencyMs = latencyMs;
        this.latencyStdDevMs = latencyStdDevMs;
        this.minStdDevs = minStdDevs;
        this.stdDevsExponent = stdDevsExponent;
        this.txtyStdDevs = txtyStdDevs;

        this.aspectRatio = (double) getDetectorWidth() / getDetectorHeight();
        double diagonalFovRad = Math.toRadians(fov);
        double halfDiagonal = Math.tan(diagonalFovRad / 2.0);

        double k = 1.0 / Math.sqrt(1 + aspectRatio * aspectRatio);
        double halfHorizontalFov = Math.atan(halfDiagonal * aspectRatio * k);
        double halfVerticalFov = Math.atan(halfDiagonal * k);

        this.horizontalFov = Math.toDegrees(halfHorizontalFov * 2.0);
        this.verticalFov = Math.toDegrees(halfVerticalFov * 2.0);
    }

    /**
     * Gets the width of the camera detector after downscaling.
     * @return The width of the camera detector in pixels.
     */
    public int getDetectorWidth() {
        return (int) Math.floor(width / downscale);
    }

    /**
     * Gets the height of the camera detector after downscaling.
     * @return The height of the camera detector in pixels.
     */
    public int getDetectorHeight() {
        return (int) Math.floor(height / downscale);
    }

    /**
     * Calculates the standard deviations for pose estimates based on average tag distance.
     * @param avgTagDist The average distance to the detected tags.
     * @return The calculated standard deviations.
     */
    public double calculateStdDevs(double avgTagDist) {
        if (avgTagDist <= 0) {
            return minStdDevs;
        }

        return minStdDevs * Math.pow(stdDevsExponent, avgTagDist);
    }

    /**
     * Checks if a point in camera-relative coordinates is within the camera's field of view.
     * @param cameraRelativePosition The position of the point relative to the camera, where +X is forward, +Y is left, and +Z is up.
     * @return True if the point is within the camera's field of view, false otherwise.
     */
    public boolean isPointInFOV(Translation3d cameraRelativePosition) {
        double horizontalAngle = Math.atan2(cameraRelativePosition.getY(), cameraRelativePosition.getX());

        double verticalAngle = Math.atan(cameraRelativePosition.getZ()
                / Math.sqrt(cameraRelativePosition.getX() * cameraRelativePosition.getX()
                        + cameraRelativePosition.getY() * cameraRelativePosition.getY()));

        return Math.abs(horizontalAngle) <= Math.toRadians(horizontalFov / 2.0)
                && Math.abs(verticalAngle) <= Math.toRadians(verticalFov / 2.0);
    }
}
