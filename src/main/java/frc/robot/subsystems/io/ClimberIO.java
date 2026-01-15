package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ClimberIO extends AutoCloseable {
    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setVoltage(double outputVolts);

    void setPosition(double newValue);

    void setMotionMagic(MotionMagicVoltage request);

    double getPosition();

    double getVelocity();

    void setPercent(double newValue);

    double getPercent();

    double getVoltage();

    void setRatchet(double value);

    double getCurrentDrawAmps();

    void simulationPeriodic();
}
