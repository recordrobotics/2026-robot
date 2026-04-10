package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.utils.TalonFXOrchestra;

public class ClimberReal implements ClimberIO {

    private final TalonFX motor;

    public ClimberReal() {
        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
        RobotContainer.orchestra.add(motor, TalonFXOrchestra.Tracks.CLIMBER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    @Override
    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.connected = motor.isConnected();
        inputs.positionMeters = motor.getPosition().getValueAsDouble();
        inputs.velocityMps = motor.getVelocity().getValueAsDouble();
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValue();
    }

    @Override
    public void close() {
        motor.close();
    }
}
