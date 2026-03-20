package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.subsystems.io.ShooterIO;

public class ShooterStub implements ShooterIO {

    @Override
    public void applyFlywheelLeaderTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void applyFlywheelFollowerTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setFlywheelFollowerControl(ControlRequest request) {
        // stub
    }

    @Override
    public void setHoodControl(ControlRequest request) {
        // stub
    }

    @Override
    public Follower createFlywheelFollower() {
        // stub
        return new Follower(-1, MotorAlignmentValue.Opposed); // invalid ID for stub
    }

    @Override
    public double getFlywheelLeaderPositionMeters() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelFollowerPositionMeters() {
        // stub
        return 0.0;
    }

    @Override
    public double getHoodPositionRotations() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelLeaderVelocityMps() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelFollowerVelocityMps() {
        // stub
        return 0.0;
    }

    @Override
    public double getHoodVelocityRotationsPerSecond() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelLeaderVoltage() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelFollowerVoltage() {
        // stub
        return 0.0;
    }

    @Override
    public double getHoodVoltage() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelLeaderCurrentDrawAmps() {
        // stub
        return 0.0;
    }

    @Override
    public double getFlywheelFollowerCurrentDrawAmps() {
        // stub
        return 0.0;
    }

    @Override
    public double getHoodCurrentDrawAmps() {
        // stub
        return 0.0;
    }

    @Override
    public void setHoodPositionRotations(double newValue) {
        // stub

    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        // stub
    }

    @Override
    public void close() {
        // stub
    }

    @Override
    public void simulationPeriodic() {
        // stub
    }
}
