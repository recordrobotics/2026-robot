package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.SpindexerIO;

public class SpindexerReal implements SpindexerIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX spindexer;

    public SpindexerReal(double periodicDt) {
        this.periodicDt = periodicDt;

        spindexer = new TalonFX(RobotMap.Spindexer.MOTOR_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        spindexer.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        spindexer.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        spindexer.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return spindexer.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return spindexer.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return spindexer.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return spindexer.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }

    @Override
    public void close() {
        spindexer.close();
    }
}
