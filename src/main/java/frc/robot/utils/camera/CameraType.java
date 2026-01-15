package frc.robot.utils.camera;

public enum CameraType {
    LIMELIGHT_2(1280, 960, 2, 74.36, 13, 0.2, 0.0005, 35, 5, 0.55, 1.4175, 0.35),
    LIMELIGHT_3G(
            1280, 960, 1, 100, 8, 0.2, 0.0005, 35, 5, 0.55, 1.11574,
            0.35), // TODO: what is the actual fov of new lens we replaced
    SVPRO_GLOBAL_SHUTTER(1920, 1200, 1, 83, 23, 0.2, 0.0005, 35, 5, 0.55, 1.3, 0.35);

    final int width;
    final int height;
    final double downscale;
    final double fov; // diagonal
    final double fps;
    final double pxError;
    final double pxErrorStdDev;
    final double latencyMs;
    final double latencyStdDevMs;

    final double minStdDevs;
    final double stdDevsExponent;
    final double txtyStdDevs;

    CameraType(
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

    public int getDetectorWidth() {
        return (int) Math.floor(width / downscale);
    }

    public int getDetectorHeight() {
        return (int) Math.floor(height / downscale);
    }

    public double calculateStdDevs(double avgTagDist) {
        if (avgTagDist <= 0) {
            return minStdDevs;
        }

        return minStdDevs * Math.pow(stdDevsExponent, avgTagDist);
    }
}
