package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public interface FeederIO {

    void applyTalonFXConfig(TalonFXConfiguration config);

    void setMotionMagic(MotionMagicVelocityVoltage request);

    void setVoltage(double newValue);

    double getPositionRotations();

    double getVelocityRotationsPerSecond();

    double getVoltage();

    double getCurrentDrawAmps();

    void simulationPeriodic();

    void close();
}
