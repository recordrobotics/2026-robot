package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberStub implements ClimberIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public void updateInputs(ClimberIO.ClimberIOInputs inputs) {}

    @Override
    public void close() {}
}
