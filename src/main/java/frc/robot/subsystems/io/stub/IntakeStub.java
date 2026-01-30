package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.subsystems.io.IntakeIO;

public class IntakeStub implements IntakeIO {
    @Override
    public void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public Follower createArmFollower() {
        return new Follower(-1, MotorAlignmentValue.Opposed); // invalid ID for stub
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setWheelVoltage(double outputVolts) {}

    @Override
    public void setArmVoltage(double outputVolts) {}

    @Override
    public void setArmLeaderMotionMagic(MotionMagicExpoVoltage request) {}

    @Override
    public void setArmFollowerMotionMagic(Follower request) {}

    @Override
    public void setWheelMotionMagic(MotionMagicVelocityVoltage request) {}

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
    public double getWheelVelocityRotationsPerSecond() {
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
    public void setWheelPositionRotations(double newValue) {}

    @Override
    public void setArmPositionRotations(double newValue) {}

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
