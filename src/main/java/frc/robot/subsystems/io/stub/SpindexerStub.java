package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.SpindexerIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class SpindexerStub implements SpindexerIO {
    public SpindexerStub() {}

    public SpindexerStub(double periodicDt) {}

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
