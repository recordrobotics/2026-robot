package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Current;
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
    public void setFlywheelPositionMeters(double newValue) {
        flywheelLeader.setPosition(newValue);
        flywheelFollower.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        hood.setPosition(newValueRotations);
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        flywheelLeader.setControl(request);
    }

    @Override
    public void setFlywheelFollowerControl(ControlRequest request) {
        flywheelFollower.setControl(request);
    }

    @Override
    public void setHoodControl(ControlRequest request) {
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
    public Current getFlywheelLeaderCurrentDraw() {
        return flywheelLeader.getSupplyCurrent().getValue();
    }

    @Override
    public Current getFlywheelFollowerCurrentDraw() {
        return flywheelFollower.getSupplyCurrent().getValue();
    }

    @Override
    public Current getHoodCurrentDraw() {
        return hood.getSupplyCurrent().getValue();
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
