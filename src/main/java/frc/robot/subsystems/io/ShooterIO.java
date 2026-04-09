package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class ShooterIOInputs {
        public double flywheelPositionMeters = 0;
        public double flywheelVelocityMps = 0;
        public double flywheelVoltage = 0;
        public Current flywheelCurrentDraw = Amps.zero();

        public boolean hoodConnected = false;
        public double hoodPositionRotations = 0;
        public double hoodVelocityRotationsPerSecond = 0;
        public double hoodVoltage = 0;
        public Current hoodCurrentDraw = Amps.zero();
    }

    void updateInputs(ShooterIOInputs inputs);

    void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration);

    void applyHoodTalonFXConfig(TalonFXConfiguration configuration);

    void setFlywheelControl(ControlRequest request);

    void setHoodControl(ControlRequest request);

    void setHoodPositionRotations(double newValue);

    void setFlywheelPositionMeters(double newValue);

    void close();

    default void simulationPeriodic() {}
}
