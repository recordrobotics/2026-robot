package frc.robot.subsystems.io.stub;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.io.NavSensorIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class NavSensorStub implements NavSensorIO {

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {}

    @Override
    public void reset() {}

    @Override
    public void resetDisplacement() {}

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.kZero;
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
        return DegreesPerSecond.of(0);
    }

    @Override
    public AngularVelocity getPitchRate() {
        return DegreesPerSecond.of(0);
    }

    @Override
    public AngularVelocity getRollRate() {
        return DegreesPerSecond.of(0);
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
        return false;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
