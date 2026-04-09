package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.NavSensorIO;
import frc.robot.utils.ManagedSubsystemBase;

public final class NavSensor extends ManagedSubsystemBase {

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

    private final Alert disconnectedAlert = new Alert("Nav Disconnected!", AlertType.kError);

    public NavSensor(NavSensorIO io) {
        this.io = io;

        io.applyPigeon2Config(new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0)));

        io.reset();
        io.resetDisplacement(); // Technically not necessary but whatever

        disconnectedAlert.set(!io.isConnected());
    }

    public boolean isConnected() {
        return io.isConnected();
    }

    public Rotation2d getYaw() {
        return io.getYaw();
    }

    public Rotation2d getPitch() {
        return io.getPitch();
    }

    public Rotation2d getRoll() {
        return io.getRoll();
    }

    public AngularVelocity getRollRate() {
        return io.getRollRate();
    }

    public AngularVelocity getPitchRate() {
        return io.getPitchRate();
    }

    public AngularVelocity getYawRate() {
        return io.getYawRate();
    }

    public double getJerkMagnitude() {
        return Math.sqrt(jerkX * jerkX + jerkY * jerkY);
    }

    @Override
    public void periodicManaged() {
        double accelX = io.getWorldLinearAccelX().in(MetersPerSecondPerSecond);
        double accelY = io.getWorldLinearAccelY().in(MetersPerSecondPerSecond);
        jerkX = (accelX - lastAccelX) / PERIODIC;
        jerkY = (accelY - lastAccelY) / PERIODIC;
        lastAccelX = accelX;
        lastAccelY = accelY;

        disconnectedAlert.set(!io.isConnected());
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
