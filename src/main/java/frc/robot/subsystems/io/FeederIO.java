package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.units.measure.Current;

public interface FeederIO {

    void applyTalonFXConfig(TalonFXConfiguration config);

    void setMotionMagic(MotionMagicVelocityVoltage request);

    void setVoltage(double newValue);

    double getPositionRotations();

    double getVelocityRotationsPerSecond();

    double getVoltage();

    Current getCurrentDraw();

    boolean isBottomBeamBroken();

    boolean isTopBeamBroken();

    void simulationPeriodic();

    void close();
}
