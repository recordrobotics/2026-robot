package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;

public interface IntakeIO {
    void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration);

    void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration);

    Follower createArmFollower();

    void applyWheelTalonFXConfig(TalonFXConfiguration configuration);

    void setArmLeaderControl(ControlRequest request);

    void setArmFollowerControl(ControlRequest request);

    void setWheelControl(ControlRequest request);

    double getArmLeaderPositionRotations();

    double getArmFollowerPositionRotations();

    double getWheelPositionMeters();

    double getArmLeaderVelocityRotationsPerSecond();

    double getArmFollowerVelocityRotationsPerSecond();

    double getWheelVelocityMps();

    double getWheelVoltage();

    double getArmLeaderVoltage();

    double getArmFollowerVoltage();

    double getWheelCurrentDrawAmps();

    double getArmLeaderCurrentDrawAmps();

    double getArmFollowerCurrentDrawAmps();

    void setWheelPositionMeters(double newValue);

    void setArmPositionRotations(double newValue);

    void close();

    void simulationPeriodic();
}
