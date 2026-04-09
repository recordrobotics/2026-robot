package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.io.ImuIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class ImuSimNavX implements ImuIO {

    private final GyroSimulation gyroSimulation;

    public ImuSimNavX(GyroSimulation gyroSimulation) {
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
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = true;
        inputs.yaw = gyroSimulation.getGyroReading();
        inputs.pitch = Rotation2d.kZero;
        inputs.roll = Rotation2d.kZero;
        inputs.yawRate = gyroSimulation.getMeasuredAngularVelocity();
        inputs.pitchRate = DegreesPerSecond.of(0);
        inputs.rollRate = DegreesPerSecond.of(0);
        inputs.worldLinearAccelX = MetersPerSecondPerSecond.of(0);
        inputs.worldLinearAccelY = MetersPerSecondPerSecond.of(0);
    }

    @Override
    public void close() {
        /* nothing to close */
    }
}
