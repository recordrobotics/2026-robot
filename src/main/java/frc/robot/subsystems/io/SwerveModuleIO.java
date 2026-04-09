package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO extends AutoCloseable {

    @AutoLog
    @SuppressWarnings("java:S1104") /* public fields in input */
    class SwerveModuleIOInputs {
        public boolean driveMotorConnected = false;
        public double driveMotorPositionMeters = 0;
        public double driveMotorVelocityMps = 0;
        public double driveMotorAccelerationMps2 = 0;
        public double driveMotorVoltage = 0;
        public Current driveMotorCurrentDraw = Amps.zero();

        public boolean turnMotorConnected = false;
        public double turnMotorPositionRotations = 0;
        public double turnMotorVelocityRps = 0;
        public double turnMotorVoltage = 0;
        public Current turnMotorCurrentDraw = Amps.zero();

        public boolean encoderConnected = false;
        public double encoderPositionRotations = 0;
        public MagnetHealthValue encoderMagnetHealth = MagnetHealthValue.Magnet_Invalid;
    }

    void updateInputs(SwerveModuleIOInputs inputs);

    void applyDriveTalonFXConfig(TalonFXConfiguration configuration);

    void applyTurnTalonFXConfig(TalonFXConfiguration configuration);

    void applyTurningEncoderConfig(CANcoderConfiguration configuration);

    void setTurnControl(ControlRequest request);

    void setDriveControl(ControlRequest request);

    void setDriveMechanismPosition(double newValue);

    void setTurnMechanismPosition(double newValue);

    void close();

    default void simulationPeriodic() {}
}
