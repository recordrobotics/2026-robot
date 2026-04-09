package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.FeederIO;

public class FeederStub implements FeederIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        // stub
    }

    @Override
    public void setControl(ControlRequest request) {
        // stub
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // stub
    }

    @Override
    public void close() {
        // stub
    }
}
