package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.NavSensorIO;
import frc.robot.utils.ManagedSubsystemBase;

public class NavSensor extends ManagedSubsystemBase {

    private static final double PERIODIC = RobotContainer.ROBOT_PERIODIC;

    private final NavSensorIO io;

    /**
     * The magnitude of a derivative of a vector is not equal to the derivative of a magnitude of a
     * vector... because If a vector is on a unit circle, its magnitude is always 1 however, that
     * vector might change its angle and that is still a change in the vector that we need so we take
     * the derivative first and then the magnitude.
     */
    private double lastAccelX;

    private double lastAccelY;

    private double jerkX;
    private double jerkY;

    // variable to keep track of a reference angle whenever you reset
    private double referenceAngle;

    public NavSensor(NavSensorIO io) {
        this.io = io;

        io.reset();
        io.resetDisplacement(); // Technically not necessary but whatever

        referenceAngle = io.getAngle();

        DashboardUI.Overview.setNavSensor(io::isConnected);
    }

    // Stores the reference angle as whatever the angle is currently measured to be
    public void resetAngleAdjustment() {
        referenceAngle = io.getAngle();
    }

    // Gets the angle minus the reference angle
    public Rotation2d getAdjustedAngle() {
        return Rotation2d.fromDegrees((io.getAngle() - referenceAngle));
    }

    public double getPitch() {
        return io.getPitch();
    }

    public double getRoll() {
        return io.getRoll();
    }

    // Gets the angular velocity in degrees per second
    public double getYawRate() {
        return io.getYawRate();
    }

    public double getJerkMagnitude() {
        return Math.sqrt(jerkX * jerkX + jerkY * jerkY);
    }

    @Override
    public void periodicManaged() {
        double accelX = io.getWorldLinearAccelX();
        double accelY = io.getWorldLinearAccelY();
        jerkX = (accelX - lastAccelX) / PERIODIC;
        jerkY = (accelY - lastAccelY) / PERIODIC;
        lastAccelX = accelX;
        lastAccelY = accelY;
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        io.close();
    }
}
