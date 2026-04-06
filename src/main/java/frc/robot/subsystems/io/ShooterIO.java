package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.units.measure.Current;

public interface ShooterIO {

    void applyFlywheelLeaderTalonFXConfig(TalonFXConfiguration configuration);

    void applyFlywheelFollowerTalonFXConfig(TalonFXConfiguration configuration);

    void applyHoodTalonFXConfig(TalonFXConfiguration configuration);

    void setFlywheelControl(ControlRequest request);

    void setFlywheelFollowerControl(ControlRequest request);

    void setHoodControl(ControlRequest request);

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

    Current getFlywheelLeaderCurrentDraw();

    Current getFlywheelFollowerCurrentDraw();

    Current getHoodCurrentDraw();

    void setHoodPositionRotations(double newValue);

    void setFlywheelPositionMeters(double newValue);

    void close();

    void simulationPeriodic();
}
