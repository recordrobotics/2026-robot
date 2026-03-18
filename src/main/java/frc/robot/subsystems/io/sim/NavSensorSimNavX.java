package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.io.NavSensorIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class NavSensorSimNavX implements NavSensorIO {

    private final GyroSimulation gyroSimulation;

    public NavSensorSimNavX(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {
        /* not supported */
    }

    @Override
    public void reset() {
        gyroSimulation.setRotation(Rotation2d.kZero);
    }

    @Override
    public void resetDisplacement() {
        /* not supported */
    }

    @Override
    public Rotation2d getYaw() {
        return gyroSimulation.getGyroReading();
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.kZero;
    }

    @Override
    public AngularVelocity getYawRate() {
        return gyroSimulation.getMeasuredAngularVelocity();
    }

    @Override
    public LinearAcceleration getWorldLinearAccelX() {
        return MetersPerSecondPerSecond.of(0);
    }

    @Override
    public LinearAcceleration getWorldLinearAccelY() {
        return MetersPerSecondPerSecond.of(0);
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void close() throws Exception {
        /* nothing to close */
    }

    @Override
    public void simulationPeriodic() {
        /* simulation handled by maplesim */
    }
}
