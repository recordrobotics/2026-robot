package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.CoralIntakeIO;

public class CoralIntakeReal implements CoralIntakeIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final SparkMax wheel;
    private final TalonFX arm;

    public CoralIntakeReal(double periodicDt) {
        this.periodicDt = periodicDt;

        wheel = new SparkMax(RobotMap.CoralIntake.WHEEL_ID, MotorType.kBrushless);
        arm = new TalonFX(RobotMap.CoralIntake.ARM_ID);
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        arm.getConfigurator().apply(configuration);
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        wheel.setVoltage(outputVolts);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        arm.setVoltage(outputVolts);
    }

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {
        arm.setControl(request);
    }

    @Override
    public void setWheelPosition(double newValue) {
        wheel.getEncoder().setPosition(newValue);
    }

    @Override
    public void setArmPosition(double newValue) {
        arm.setPosition(newValue);
    }

    @Override
    public double getWheelPosition() {
        return wheel.getEncoder().getPosition();
    }

    @Override
    public double getWheelVelocity() {
        return wheel.getEncoder().getVelocity();
    }

    @Override
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public double getWheelVoltage() {
        return wheel.getAppliedOutput() * wheel.getBusVoltage();
    }

    @Override
    public double getArmVoltage() {
        return arm.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmVelocity() {
        return arm.getVelocity().getValueAsDouble();
    }

    @Override
    public void setWheelPercent(double newValue) {
        wheel.set(newValue);
    }

    @Override
    public void setArmPercent(double newValue) {
        arm.set(newValue);
    }

    @Override
    public double getWheelPercent() {
        return wheel.get();
    }

    @Override
    public double getArmPercent() {
        return arm.get();
    }

    @Override
    public double getWheelCurrentDrawAmps() {
        return wheel.getOutputCurrent();
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return arm.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        wheel.close();
        arm.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
