package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.TurretIO;

public class TurretStub implements TurretIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setPositionRotations(double newValue) {
        // stub
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // stub
    }

    @Override
    public void close() {
        // stub
    }
}
