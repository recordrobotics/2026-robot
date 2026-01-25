package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.IntakeIO;

public class IntakeReal implements IntakeIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX wheelLeader;
    private final TalonFX armLeader;
    private final TalonFX armFollower;

    public IntakeReal(double periodicDt) {
        this.periodicDt = periodicDt;

        wheelLeader = new TalonFX(RobotMap.Intake.WHEEL_ID);
        armLeader = new TalonFX(RobotMap.Intake.ARM_LEADER_ID);
        armFollower = new TalonFX(RobotMap.Intake.ARM_FOLLOWER_ID);
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        armLeader.getConfigurator().apply(configuration);
        armFollower.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createArmFollower() {
        return new Follower(
                RobotMap.Intake.ARM_LEADER_ID, MotorAlignmentValue.Opposed); // motors face in opposite directions
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheelLeader.getConfigurator().apply(configuration);
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        wheelLeader.setVoltage(outputVolts);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        armLeader.setVoltage(outputVolts);
    }

    @Override
    public void setArmLeaderMotionMagic(MotionMagicExpoVoltage request) {
        armLeader.setControl(request);
    }

    @Override
    public void setArmFollowerMotionMagic(Follower request) {
        armFollower.setControl(request);
    }

    @Override
    public void setWheelMotionMagic(MotionMagicVelocityVoltage request) {
        wheelLeader.setControl(request);
    }

    @Override
    public void setWheelPositionRotations(double newValue) {
        wheelLeader.setPosition(newValue);
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        armLeader.setPosition(newValue);
        armFollower.setPosition(newValue);
    }

    @Override
    public double getWheelVelocityRotationsPerSecond() {
        return wheelLeader.getVelocity().getValueAsDouble();
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
    public double getWheelPositionRotations() {
        return wheelLeader.getPosition().getValueAsDouble();
    }

    @Override
    public double getWheelVoltage() {
        return wheelLeader.getMotorVoltage().getValueAsDouble();
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
    public double getArmLeaderVelocityRotationsPerSecond() {
        return armLeader.getVelocity().getValueAsDouble();
    }

    @Override
    public double getArmFollowerVelocityRotationsPerSecond() {
        return armFollower.getVelocity().getValueAsDouble();
    }

    @Override
    public double getWheelCurrentDrawAmps() {
        return wheelLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getArmLeaderCurrentDrawAmps() {
        return armLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getArmFollowerCurrentDrawAmps() {
        return armFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        wheelLeader.close();
        armLeader.close();
        armFollower.close();
    }

    @Override
    public void simulationPeriodic() {
        // This is IntakeReal, no simulation needed
    }
}
