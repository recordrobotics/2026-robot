package frc.robot.subsystems.io.stub;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;
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
    public boolean hasHitForwardSoftLimit() {
        return false;
    }

    @Override
    public boolean hasHitReverseSoftLimit() {
        return false;
    }

    @Override
    public double getPositionRotations() {
        // stub
        return 0;
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        // stub
        return 0;
    }

    @Override
    public double getVoltage() {
        // stub
        return 0;
    }

    @Override
    public Current getCurrentDraw() {
        // stub
        return Amps.zero();
    }

    @Override
    public void setPositionRotations(double newValue) {
        // stub
    }

    @Override
    public LimitSwitchStates getLimitSwitchStates() {
        // stub
        return LimitSwitchStates.NO_HITS;
    }

    @Override
    public void close() {
        // stub
    }

    @Override
    public void simulationPeriodic() {
        // stub
    }
}
