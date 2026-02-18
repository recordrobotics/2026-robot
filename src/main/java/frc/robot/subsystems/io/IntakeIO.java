package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public interface IntakeIO {
    void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration);

    void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration);

    Follower createArmFollower();

    void applyWheelTalonFXConfig(TalonFXConfiguration configuration);

    void setArmLeaderMotionMagic(MotionMagicExpoVoltage request); // test

    void setArmFollowerMotionMagic(Follower request);

    void setWheelMotionMagic(MotionMagicVelocityVoltage request);

    void setArmVoltage(double newValue); // follower will mirror leader voltage

    void setWheelVoltage(double newValue);

    double getArmLeaderPositionRotations();

    double getArmFollowerPositionRotations();

    double getWheelPositionRotations();

    double getArmLeaderVelocityRotationsPerSecond();

    double getArmFollowerVelocityRotationsPerSecond();

    double getWheelVelocityMps();

    double getWheelVoltage();

    double getArmLeaderVoltage();

    double getArmFollowerVoltage();

    double getWheelCurrentDrawAmps();

    double getArmLeaderCurrentDrawAmps();

    double getArmFollowerCurrentDrawAmps();

    void setWheelPositionMps(double newValue);

    void setArmPositionRotations(double newValue);

    void close();

    void simulationPeriodic();
}
