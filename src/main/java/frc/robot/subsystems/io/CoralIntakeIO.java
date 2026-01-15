package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface CoralIntakeIO extends AutoCloseable {

    void applyArmTalonFXConfig(TalonFXConfiguration configuration);

    void setWheelVoltage(double outputVolts);

    void setArmVoltage(double outputVolts);

    void setWheelPosition(double newValue);

    void setArmPosition(double newValue);

    void setArmMotionMagic(MotionMagicExpoVoltage request);

    double getWheelPosition();

    double getWheelVelocity();

    double getWheelVoltage();

    double getArmVoltage();

    double getArmPosition();

    double getArmVelocity();

    void setWheelPercent(double newValue);

    void setArmPercent(double newValue);

    double getWheelPercent();

    double getArmPercent();

    double getWheelCurrentDrawAmps();

    double getArmCurrentDrawAmps();

    void simulationPeriodic();
}
