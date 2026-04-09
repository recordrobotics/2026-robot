package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.IntakeIO;

public class IntakeStub implements IntakeIO {
    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setArmControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setWheelControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        // stub
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        // stub
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // stub
    }

    @Override
    public void close() {
        // stub
    }
}
