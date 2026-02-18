package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;

public class ShooterReal implements ShooterIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX flywheelLeader;
    private final TalonFX flywheelFollower;
    private final TalonFX hood;

    public ShooterReal(double periodicDt) {
        this.periodicDt = periodicDt;

        flywheelLeader = new TalonFX(RobotMap.Shooter.FLYWHEEL_LEADER_ID);
        flywheelFollower = new TalonFX(RobotMap.Shooter.FLYWHEEL_FOLLOWER_ID);
        hood = new TalonFX(RobotMap.Shooter.HOOD_ID);
    }

    @Override
    public Follower createFlywheelFollower() {
        return new Follower(
                RobotMap.Shooter.FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed); // motors face in opposite directions
    }

    @Override
    public void applyFlywheelLeaderTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelLeader.getConfigurator().apply(configuration);
    }

    @Override
    public void applyFlywheelFollowerTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelFollower.getConfigurator().apply(configuration);
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        hood.getConfigurator().apply(configuration);
    }

    @Override
    public void setHoodVoltage(double outputVolts) {
        hood.setVoltage(outputVolts);
    }

    @Override
    public void setFlywheelVoltage(double outputVolts) {
        flywheelLeader.setVoltage(outputVolts);
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        flywheelLeader.setPosition(newValue);
        flywheelFollower.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        hood.setPosition(newValueRotations);
    }

    @Override
    public void setFlywheelMotionMagic(MotionMagicVelocityVoltage request) {
        flywheelLeader.setControl(request);
    }

    @Override
    public void setFlywheelFollowerMotionMagic(Follower request) {
        flywheelFollower.setControl(request);
    }

    @Override
    public void setHoodMotionMagic(MotionMagicExpoVoltage request) {
        hood.setControl(request);
    }

    @Override
    public double getFlywheelLeaderPositionMeters() {
        return flywheelLeader.getPosition().getValueAsDouble();
    }

    @Override
    public double getFlywheelFollowerPositionMeters() {
        return flywheelFollower.getPosition().getValueAsDouble();
    }

    @Override
    public double getFlywheelLeaderVelocityMps() {
        return flywheelLeader.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFlywheelFollowerVelocityMps() {
        return flywheelFollower.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFlywheelLeaderVoltage() {
        return flywheelLeader.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getFlywheelFollowerVoltage() {
        return flywheelFollower.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getHoodVoltage() {
        return hood.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getHoodPositionRotations() {
        return hood.getPosition().getValueAsDouble();
    }

    @Override
    public double getHoodVelocityRotationsPerSecond() {
        return hood.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFlywheelLeaderCurrentDrawAmps() {
        return flywheelLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getFlywheelFollowerCurrentDrawAmps() {
        return flywheelFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getHoodCurrentDrawAmps() {
        return hood.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() {
        flywheelLeader.close();
        flywheelFollower.close();
        hood.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
