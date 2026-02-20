package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface ClimberIO extends AutoCloseable {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setMotorVoltage(double outputVolts);

    double getMotorVoltage();

    void setMotorPosition(double newValue);

    void setMotionMagic(MotionMagicExpoVoltage request);

    double getMotorPosition();

    double getMotorVelocity();

    void setMotorPercent(double newValue);

    double getMotorPercent();

    double getMotorCurrentDraw();

    void simulationPeriodic();
}
