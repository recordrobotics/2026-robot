package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public interface SwerveModuleIO extends AutoCloseable {

    void applyDriveTalonFXConfig(TalonFXConfiguration configuration);

    void applyTurnTalonFXConfig(TalonFXConfiguration configuration);

    void setDriveMotorVoltage(double newValue);

    void setTurnMotorVoltage(double newValue);

    void setTurnMotorMotionMagic(MotionMagicExpoVoltage request);

    void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request);

    double getDriveMotorVoltage();

    double getTurnMotorVoltage();

    void setDriveMotorPercent(double newValue);

    void setTurnMotorPercent(double newValue);

    double getDriveMotorPercent();

    double getTurnMotorPercent();

    double getAbsoluteEncoder();

    double getTurnMechanismPosition();

    double getTurnMechanismVelocity();

    double getDriveMechanismPosition();

    double getDriveMechanismVelocity();

    double getDriveMechanismAcceleration();

    void setDriveMechanismPosition(double newValue);

    void setTurnMechanismPosition(double newValue);

    double getDriveMotorCurrentDrawAmps();

    double getTurnMotorCurrentDrawAmps();

    void simulationPeriodic();
}
