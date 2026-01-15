package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberReal implements ClimberIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX motor;
    private final Servo ratchet;

    public ClimberReal(double periodicDt) {
        this.periodicDt = periodicDt;

        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
        ratchet = new Servo(RobotMap.Climber.RATCHET_SERVO_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setMotionMagic(MotionMagicVoltage request) {
        motor.setControl(request);
    }

    @Override
    public void setVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }

    @Override
    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setPercent(double newValue) {
        motor.set(newValue);
    }

    @Override
    public double getPercent() {
        return motor.get();
    }

    @Override
    public double getCurrentDrawAmps() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setRatchet(double value) {
        ratchet.set(value);
    }

    @Override
    public void close() throws Exception {
        motor.close();
        ratchet.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
