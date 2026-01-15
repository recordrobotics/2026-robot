package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface ElevatorArmIO extends AutoCloseable {

    void applyArmTalonFXConfig(TalonFXConfiguration configuration);

    void setArmVoltage(double outputVolts);

    void setArmPosition(double newValue);

    void setArmMotionMagic(MotionMagicExpoVoltage request);

    double getArmPosition();

    double getArmVelocity();

    void setArmPercent(double newValue);

    double getArmPercent();

    double getArmVoltage();

    double getArmCurrentDrawAmps();

    void simulationPeriodic();
}
