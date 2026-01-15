package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface ElevatorIO extends AutoCloseable {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    Follower createFollower();

    void setLeadMotorVoltage(double outputVolts);

    double getLeadMotorVoltage();

    double getFollowerMotorVoltage();

    void setLeadMotorPosition(double newValue);

    void setFollowerMotorPosition(double newValue);

    void setLeadMotionMagic(MotionMagicExpoVoltage request);

    void setFollowerMotionMagic(Follower request);

    double getLeadMotorPosition();

    double getLeadMotorVelocity();

    double getFollowerMotorPosition();

    double getFollowerMotorVelocity();

    void setLeadMotorPercent(double newValue);

    double getLeadMotorPercent();

    boolean isTopEndStopPressed();

    boolean isBottomEndStopPressed();

    double getLeadMotorCurrentDraw();

    double getFollowerMotorCurrentDraw();

    void simulationPeriodic();
}
