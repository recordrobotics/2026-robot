package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class ClimberIOInputs {
        public boolean connected = false;
        public double positionMeters = 0;
        public double velocityMps = 0;
        public double voltage = 0;
        public Current currentDraw = Amps.zero();
    }

    void updateInputs(ClimberIOInputs inputs);

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setPosition(double newValue);

    void setControl(ControlRequest request);

    void close();

    default void simulationPeriodic() {}
}
