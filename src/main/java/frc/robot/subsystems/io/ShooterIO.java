package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public interface ShooterIO {

    void applyFlywheelLeaderTalonFXConfig(TalonFXConfiguration configuration);

    void applyFlywheelFollowerTalonFXConfig(TalonFXConfiguration configuration);

    void applyHoodTalonFXConfig(TalonFXConfiguration configuration);

    void setHoodVoltage(double outputVolts);

    void setFlywheelVoltage(double outputVolts);

    void setFlywheelMotionMagic(MotionMagicVelocityVoltage request);

    void setFlywheelFollowerMotionMagic(Follower request);

    void setHoodMotionMagic(MotionMagicExpoVoltage request);

    Follower createFlywheelFollower();

    double getFlywheelLeaderPositionMeters();

    double getFlywheelFollowerPositionMeters();

    double getHoodPositionRotations();

    double getFlywheelLeaderVelocityMps();

    double getFlywheelFollowerVelocityMps();

    double getHoodVelocityRotationsPerSecond();

    double getFlywheelLeaderVoltage();

    double getFlywheelFollowerVoltage();

    double getHoodVoltage();

    double getFlywheelLeaderCurrentDrawAmps();

    double getFlywheelFollowerCurrentDrawAmps();

    double getHoodCurrentDrawAmps();

    void setHoodPositionRotations(double newValue);

    void setFlywheelPositionMeters(double newValue);

    void close();

    void simulationPeriodic();
}
