package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.TurretIO;

public class TurretStub implements TurretIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {
        // stub
    }

    @Override
    public void setVoltage(double newValue) {
        // stub
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
    public double getCurrentDrawAmps() {
        // stub
        return 0;
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
