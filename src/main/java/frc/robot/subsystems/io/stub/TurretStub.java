package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.TurretIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class TurretStub implements TurretIO {
    public TurretStub() {}

    public TurretStub(double periodicDt) {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void setPositionRotations(double newValue) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
