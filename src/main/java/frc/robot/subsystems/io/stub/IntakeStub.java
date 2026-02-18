package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.subsystems.io.IntakeIO;

public class IntakeStub implements IntakeIO {
    @Override
    public void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public Follower createArmFollower() {
        return new Follower(-1, MotorAlignmentValue.Opposed); // invalid ID for stub
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        // stub
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        // stub
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        // stub
    }

    @Override
    public void setArmLeaderMotionMagic(MotionMagicExpoVoltage request) {
        // stub
    }

    @Override
    public void setArmFollowerMotionMagic(Follower request) {
        // stub
    }

    @Override
    public void setWheelMotionMagic(MotionMagicVelocityVoltage request) {
        // stub
    }

    @Override
    public double getArmLeaderPositionRotations() {
        return 0.0;
    }

    @Override
    public double getArmFollowerPositionRotations() {
        return 0.0;
    }

    @Override
    public double getWheelPositionRotations() {
        return 0.0;
    }

    @Override
    public double getArmLeaderVelocityRotationsPerSecond() {
        return 0.0;
    }

    @Override
    public double getArmFollowerVelocityRotationsPerSecond() {
        return 0.0;
    }

    @Override
    public double getWheelVelocityMps() {
        return 0.0;
    }

    @Override
    public double getWheelVoltage() {
        return 0.0;
    }

    @Override
    public double getArmLeaderVoltage() {
        return 0.0;
    }

    @Override
    public double getArmFollowerVoltage() {
        return 0.0;
    }

    @Override
    public double getWheelCurrentDrawAmps() {
        return 0.0;
    }

    @Override
    public double getArmLeaderCurrentDrawAmps() {
        return 0.0;
    }

    @Override
    public double getArmFollowerCurrentDrawAmps() {
        return 0.0;
    }

    @Override
    public void setWheelPositionMps(double newValue) {
        // stub
    }

    @Override
    public void setArmPositionRotations(double newValue) {
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
