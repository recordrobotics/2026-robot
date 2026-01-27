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
    SVPRO_GLOBAL_SHUTTER(1920, 1200, 1, 83, 23, 0.2, 0.0005, 35, 5, 0.55, 1.3, 0.35);

    private static final double NEAR_PLANE = 0.01; // meters

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
     * Calculates the projected area percentage of a sphere onto the camera's near plane.
     * <p>Note: it is simplified and does not clip area by the FOV bounds
     * <p>However, if the sphere is completely outside the FOV, it returns 0.
     * @param cameraRelativePosition The position of the sphere relative to the camera.
     * @param radius The radius of the sphere.
     * @return The projected area percentage of the sphere on the near plane.
     */
    public double calculateSphereProjectedAreaPercentage(Translation3d cameraRelativePosition, double radius) {
        if (radius <= 0) {
            return 0.0;
        }

        double depth = cameraRelativePosition.getZ();
        if (depth <= 0) {
            return 0.0;
        }

        double x = cameraRelativePosition.getX();
        double y = cameraRelativePosition.getY();
        double aspectRatio = (double) getDetectorWidth() / getDetectorHeight();
        double diagonalFovRad = Math.toRadians(fov);
        double halfDiagonal = NEAR_PLANE * Math.tan(diagonalFovRad / 2.0);

        double halfHeight = halfDiagonal / Math.sqrt(1.0 + (aspectRatio * aspectRatio));
        double halfWidth = halfHeight * aspectRatio;
        double halfVerticalFov = Math.atan2(halfHeight, NEAR_PLANE);
        double halfHorizontalFov = Math.atan2(halfWidth, NEAR_PLANE);
        double angularRadius = Math.atan2(radius, depth);
        double horizontalAngle = Math.atan2(x, depth);
        double verticalAngle = Math.atan2(y, depth);

        if (Math.abs(horizontalAngle) > (halfHorizontalFov + angularRadius)
                || Math.abs(verticalAngle) > (halfVerticalFov + angularRadius)) {
            return 0.0;
        }

        double nearPlaneArea = (2.0 * halfWidth) * (2.0 * halfHeight);
        double projectedRadius = (NEAR_PLANE * radius) / depth;
        double projectedArea = Math.PI * projectedRadius * projectedRadius;
        double percentage = projectedArea / nearPlaneArea;

        return Math.max(0.0, Math.min(1.0, percentage));
    }
}
