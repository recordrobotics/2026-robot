package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ImuIO;
import frc.robot.subsystems.io.ImuIOInputsAutoLogged;
import frc.robot.utils.ManagedSubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Imu extends ManagedSubsystemBase {

    private static final double PERIODIC = RobotContainer.ROBOT_PERIODIC;

    private final ImuIO io;
    private final ImuIOInputsAutoLogged inputs = new ImuIOInputsAutoLogged();

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

    private final Alert disconnectedAlert = new Alert("Imu disconnected!", AlertType.kError);

    public Imu(ImuIO io) {
        this.io = io;

        io.applyPigeon2Config(new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0)));

        io.reset();
        io.resetDisplacement(); // Technically not necessary but whatever
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    public Rotation2d getPitch() {
        return inputs.pitch;
    }

    public Rotation2d getRoll() {
        return inputs.roll;
    }

    public AngularVelocity getRollRate() {
        return inputs.rollRate;
    }

    public AngularVelocity getPitchRate() {
        return inputs.pitchRate;
    }

    public AngularVelocity getYawRate() {
        return inputs.yawRate;
    }

    public double getJerkMagnitude() {
        return Math.sqrt(jerkX * jerkX + jerkY * jerkY);
    }

    @Override
    public void periodicManaged() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Imu", inputs);

        double accelX = inputs.worldLinearAccelX.in(MetersPerSecondPerSecond);
        double accelY = inputs.worldLinearAccelY.in(MetersPerSecondPerSecond);
        jerkX = (accelX - lastAccelX) / PERIODIC;
        jerkY = (accelY - lastAccelY) / PERIODIC;
        lastAccelX = accelX;
        lastAccelY = accelY;

        disconnectedAlert.set(!inputs.connected);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }
}
