package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
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
    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
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
    public Current getCurrentDraw() {
        return motor.getSupplyCurrent().getValue();
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
