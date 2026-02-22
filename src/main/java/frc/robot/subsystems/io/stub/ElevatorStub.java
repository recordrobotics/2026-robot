package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.ClimberIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ElevatorStub implements ClimberIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ElevatorStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setVoltage(double outputVolts) {}

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {}

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
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
