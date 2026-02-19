package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.FeederIO;

public class FeederReal implements FeederIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX feeder;

    public FeederReal(double periodicDt) {
        this.periodicDt = periodicDt;

        feeder = new TalonFX(RobotMap.Feeder.MOTOR_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        feeder.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        feeder.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        feeder.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return feeder.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return feeder.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return feeder.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return feeder.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }

    @Override
    public void close() {
        feeder.close();
    }
}
