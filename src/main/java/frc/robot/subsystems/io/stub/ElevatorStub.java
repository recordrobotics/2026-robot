package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.ElevatorIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ElevatorStub implements ElevatorIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ElevatorStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public Follower createFollower() {
        return new Follower(0, false);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setLeadMotorVoltage(double outputVolts) {}

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {}

    @Override
    public void setFollowerMotionMagic(Follower request) {}

    @Override
    public double getLeadMotorVoltage() {
        return 0;
    }

    @Override
    public double getFollowerMotorVoltage() {
        return 0;
    }

    @Override
    public void setLeadMotorPosition(double newValue) {}

    @Override
    public void setFollowerMotorPosition(double newValue) {}

    @Override
    public double getLeadMotorPosition() {
        return 0;
    }

    @Override
    public double getLeadMotorVelocity() {
        return 0;
    }

    @Override
    public double getFollowerMotorPosition() {
        return 0;
    }

    @Override
    public double getFollowerMotorVelocity() {
        return 0;
    }

    @Override
    public void setLeadMotorPercent(double newValue) {}

    @Override
    public double getLeadMotorPercent() {
        return 0;
    }

    @Override
    public boolean isTopEndStopPressed() {
        return false;
    }

    @Override
    public boolean isBottomEndStopPressed() {
        return false;
    }

    @Override
    public double getLeadMotorCurrentDraw() {
        return 0;
    }

    @Override
    public double getFollowerMotorCurrentDraw() {
        return 0;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
