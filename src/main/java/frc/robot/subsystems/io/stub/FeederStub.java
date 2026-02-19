package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import frc.robot.subsystems.io.FeederIO;

public class FeederStub implements FeederIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        // stub
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        // stub
    }

    @Override
    public void setVoltage(double newValue) {
        // stub
    }

    @Override
    public double getPositionRotations() {
        // stub
        return 0.0;
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        // stub
        return 0.0;
    }

    @Override
    public double getVoltage() {
        // stub
        return 0.0;
    }

    @Override
    public double getCurrentDrawAmps() {
        // stub
        return 0.0;
    }

    @Override
    public void simulationPeriodic() {
        // stub
    }

    @Override
    public void close() {
        // stub
    }
}
