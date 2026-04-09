package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class SpindexerIOInputs {
        public boolean connected = false;
        public double positionRotations = 0;
        public double velocityRotationsPerSecond = 0;
        public double voltage = 0;
        public Current currentDraw = Amps.zero();
    }

    void updateInputs(SpindexerIOInputs inputs);

    void applyTalonFXConfig(TalonFXConfiguration config);

    void setControl(ControlRequest request);

    void close();

    default void simulationPeriodic() {}
}
