package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.SpindexerIO;

public class SpindexerReal implements SpindexerIO {

    private final TalonFX spindexer;

    public SpindexerReal() {
        spindexer = new TalonFX(RobotMap.Spindexer.MOTOR_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        spindexer.getConfigurator().apply(config);
    }

    @Override
    public void setControl(ControlRequest request) {
        spindexer.setControl(request);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.connected = spindexer.isConnected();
        inputs.positionRotations = spindexer.getPosition().getValueAsDouble();
        inputs.velocityRotationsPerSecond = spindexer.getVelocity().getValueAsDouble();
        inputs.voltage = spindexer.getMotorVoltage().getValueAsDouble();
        inputs.currentDraw = spindexer.getSupplyCurrent().getValue();
    }

    @Override
    public void close() {
        spindexer.close();
    }
}
