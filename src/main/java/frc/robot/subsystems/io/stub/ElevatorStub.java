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
    public void setMotorVoltage(double outputVolts) {}

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {}

    @Override
    public double getMotorVoltage() {
        return 0;
    }

    @Override
    public void setMotorPosition(double newValue) {}

    @Override
    public double getMotorPosition() {
        return 0;
    }

    @Override
    public double getMotorVelocity() {
        return 0;
    }

    @Override
    public void setMotorPercent(double newValue) {}

    @Override
    public double getMotorPercent() {
        return 0;
    }

    @Override
    public double getMotorCurrentDraw() {
        return 0;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
