package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.ShooterIO;

public class ShooterStub implements ShooterIO {

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setHoodControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setHoodPositionRotations(double newValue) {
        // stub

    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        // stub
    }

    @Override
    public void close() {
        // stub
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // stub
    }
}
