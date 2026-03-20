package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

public interface ClimberIO extends AutoCloseable {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    double getVoltage();

    void setPosition(double newValue);

    void setControl(ControlRequest request);

    double getPosition();

    double getVelocity();

    double getCurrentDraw();

    void simulationPeriodic();
}
