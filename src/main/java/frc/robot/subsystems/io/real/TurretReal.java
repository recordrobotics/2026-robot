package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.TurretIO;

public class TurretReal implements TurretIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX turret;

    private final DigitalInput frontLeftLimitSwitch = new DigitalInput(RobotMap.Turret.FRONT_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backLeftLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backRightLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_RIGHT_LIMIT_SWITCH_ID);

    public TurretReal(double periodicDt) {
        this.periodicDt = periodicDt;

        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        turret.getConfigurator().apply(configuration);
    }

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {
        turret.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        turret.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return turret.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return turret.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return turret.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return turret.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setPositionRotations(double newValue) {
        turret.setPosition(newValue);
    }

    @Override
    public LimitSwitchStates getLimitSwitchStates() {
        return new LimitSwitchStates(
                !frontLeftLimitSwitch.get(), !backLeftLimitSwitch.get(), !backRightLimitSwitch.get());
    }

    @Override
    public void close() {
        turret.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
