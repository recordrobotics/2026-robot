package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.ClimberIO;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ClimberStub implements ClimberIO {
    public ClimberStub() {}

    public ClimberStub(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {}

    @Override
    public void updateInputs(ClimberIOInputs inputs) {}

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public void setControl(ControlRequest request) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
