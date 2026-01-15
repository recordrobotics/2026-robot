package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorArmIO;

public class ElevatorArmReal implements ElevatorArmIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX arm;

    public ElevatorArmReal(double periodicDt) {
        this.periodicDt = periodicDt;

        arm = new TalonFX(RobotMap.ElevatorArm.ARM_ID);
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        arm.getConfigurator().apply(configuration);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        arm.setVoltage(outputVolts);
    }

    @Override
    public void setArmPosition(double newValue) {
        arm.setPosition(newValue);
    }

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {
        arm.setControl(request);
    }

    @Override
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmVelocity() {
        return arm.getVelocity().getValueAsDouble();
    }

    @Override
    public void setArmPercent(double newValue) {
        arm.set(newValue);
    }

    @Override
    public double getArmPercent() {
        return arm.get();
    }

    @Override
    public double getArmVoltage() {
        return arm.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return arm.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        arm.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
