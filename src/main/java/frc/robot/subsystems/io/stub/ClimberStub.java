package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberStub implements ClimberIO {

    public ClimberStub() {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public void setPosition(double newValue) {}

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getCurrentDraw() {
        return 0;
    }

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {
        /* stub */
    }
}
