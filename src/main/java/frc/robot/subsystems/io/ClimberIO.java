package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface ClimberIO extends AutoCloseable {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setVoltage(double outputVolts);

    double getVoltage();

    void setPosition(double newValue);

    void setMotionMagic(MotionMagicExpoVoltage request);

    double getPosition();

    double getVelocity();

    double getCurrentDraw();

    void simulationPeriodic();
}
