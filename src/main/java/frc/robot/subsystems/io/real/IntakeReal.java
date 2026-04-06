package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.IntakeIO;

public class IntakeReal implements IntakeIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX wheel;
    private final TalonFX armLeader;
    private final TalonFX armFollower;

    public IntakeReal(double periodicDt) {
        this.periodicDt = periodicDt;

        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);
        armLeader = new TalonFX(RobotMap.Intake.ARM_LEADER_ID);
        armFollower = new TalonFX(RobotMap.Intake.ARM_FOLLOWER_ID);
    }

    @Override
    public void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration) {
        armLeader.getConfigurator().apply(configuration);
    }

    @Override
    public void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration) {
        armFollower.getConfigurator().apply(configuration);
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheel.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createArmFollower() {
        return new Follower(
                RobotMap.Intake.ARM_LEADER_ID, MotorAlignmentValue.Opposed); // motors face in opposite directions
    }

    @Override
    public void setArmLeaderControl(ControlRequest request) {
        armLeader.setControl(request);
    }

    @Override
    public void setArmFollowerControl(ControlRequest request) {
        armFollower.setControl(request);
    }

    @Override
    public void setWheelControl(ControlRequest request) {
        wheel.setControl(request);
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        armLeader.setPosition(newValue);
        armFollower.setPosition(newValue);
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        wheel.setPosition(newValue);
    }

    @Override
    public double getArmLeaderPositionRotations() {
        return armLeader.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmFollowerPositionRotations() {
        return armFollower.getPosition().getValueAsDouble();
    }

    @Override
    public double getWheelPositionMeters() {
        return wheel.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmLeaderVoltage() {
        return armLeader.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmFollowerVoltage() {
        return armFollower.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getWheelVoltage() {
        return wheel.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmLeaderVelocityRotationsPerSecond() {
        return armLeader.getVelocity().getValueAsDouble();
    }

    @Override
    public double getArmFollowerVelocityRotationsPerSecond() {
        return armFollower.getVelocity().getValueAsDouble();
    }

    @Override
    public double getWheelVelocityMps() {
        return wheel.getVelocity().getValueAsDouble();
    }

    @Override
    public Current getArmLeaderCurrentDraw() {
        return armLeader.getSupplyCurrent().getValue();
    }

    @Override
    public Current getArmFollowerCurrentDraw() {
        return armFollower.getSupplyCurrent().getValue();
    }

    @Override
    public Current getWheelCurrentDraw() {
        return wheel.getSupplyCurrent().getValue();
    }

    @Override
    public void close() {
        wheel.close();
        armLeader.close();
        armFollower.close();
    }

    @Override
    public void simulationPeriodic() {
        // This is IntakeReal, no simulation needed
    }
}
