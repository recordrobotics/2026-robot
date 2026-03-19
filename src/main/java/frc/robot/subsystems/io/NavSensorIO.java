package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

public interface NavSensorIO extends AutoCloseable {

    void applyPigeon2Config(Pigeon2Configuration config);

    void reset();

    void resetDisplacement();

    Rotation2d getYaw();

    AngularVelocity getRollRate();

    AngularVelocity getPitchRate();

    AngularVelocity getYawRate();

    Rotation2d getPitch();

    Rotation2d getRoll();

    LinearAcceleration getWorldLinearAccelX();

    LinearAcceleration getWorldLinearAccelY();

    boolean isConnected();

    void simulationPeriodic();
}
