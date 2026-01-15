package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.ElevatorArmIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ElevatorArmStub implements ElevatorArmIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ElevatorArmStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setArmVoltage(double outputVolts) {}

    @Override
    public void setArmPosition(double newValue) {}

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {}

    @Override
    public double getArmPosition() {
        return 0;
    }

    @Override
    public double getArmVelocity() {
        return 0;
    }

    @Override
    public void setArmPercent(double newValue) {}

    @Override
    public double getArmPercent() {
        return 0;
    }

    @Override
    public double getArmVoltage() {
        return 0;
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return 0;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
