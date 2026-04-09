package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleReal implements SwerveModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteTurningMotorEncoder;

    public SwerveModuleReal(ModuleConstants m) {
        driveMotor = new TalonFX(m.driveMotorChannel());
        turningMotor = new TalonFX(m.turningMotorChannel());
        absoluteTurningMotorEncoder = new CANcoder(m.absoluteTurningMotorEncoderChannel());
    }

    @Override
    public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {
        driveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {
        turningMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurningEncoderConfig(CANcoderConfiguration configuration) {
        absoluteTurningMotorEncoder.getConfigurator().apply(configuration);
    }

    @Override
    public void setDriveControl(ControlRequest request) {
        driveMotor.setControl(request);
    }

    @Override
    public void setTurnControl(ControlRequest request) {
        turningMotor.setControl(request);
    }

    @Override
    public void setDriveMechanismPosition(double newValue) {
        driveMotor.setPosition(newValue);
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        turningMotor.setPosition(newValue);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorConnected = driveMotor.isConnected();
        inputs.driveMotorPositionMeters = driveMotor.getPosition().getValueAsDouble();
        inputs.driveMotorVelocityMps = driveMotor.getVelocity().getValueAsDouble();
        inputs.driveMotorAccelerationMps2 = driveMotor.getAcceleration().getValueAsDouble();
        inputs.driveMotorVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveMotorCurrentDraw = driveMotor.getSupplyCurrent().getValue();

        inputs.turnMotorConnected = turningMotor.isConnected();
        inputs.turnMotorPositionRotations = turningMotor.getPosition().getValueAsDouble();
        inputs.turnMotorVelocityRps = turningMotor.getVelocity().getValueAsDouble();
        inputs.turnMotorVoltage = turningMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnMotorCurrentDraw = turningMotor.getSupplyCurrent().getValue();

        inputs.encoderConnected = absoluteTurningMotorEncoder.isConnected();
        inputs.encoderPositionRotations =
                absoluteTurningMotorEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.encoderMagnetHealth =
                absoluteTurningMotorEncoder.getMagnetHealth().getValue();
    }

    @Override
    public void close() {
        driveMotor.close();
        turningMotor.close();
        absoluteTurningMotorEncoder.close();
    }
}
