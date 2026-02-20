package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberReal implements ClimberIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX motor;

    public ClimberReal(double periodicDt) {
        this.periodicDt = periodicDt;

        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setMotorVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {
        motor.setControl(request);
    }

    @Override
    public double getMotorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setMotorPosition(double newValue) {
        motor.setPosition(newValue);
    }

    @Override
    public double getMotorPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setMotorPercent(double newValue) {
        motor.set(newValue);
    }

    @Override
    public double getMotorPercent() {
        return motor.get();
    }

    @Override
    public double getMotorCurrentDraw() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
