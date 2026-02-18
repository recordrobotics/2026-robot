package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;

public class ShooterSim implements ShooterIO {

    private static final double HOOD_ANGLE_OFFSET = Math.PI / 2;

    private final double periodicDt;

    private final TalonFX flywheelLeader;
    private final TalonFX flywheelFollower;
    private final TalonFX hood;

    private final TalonFXSimState flywheelSimLeader;
    private final TalonFXSimState flywheelSimFollower;
    private final TalonFXSimState hoodSim;

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(2);
    private final DCMotor hoodMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim hoodSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(hoodMotor, 0.007440336, Constants.Shooter.HOOD_GEAR_RATIO),
            hoodMotor,
            Constants.Shooter.HOOD_GEAR_RATIO,
            Units.inchesToMeters(6.02),
            Constants.Shooter.HOOD_MIN_POSITION_RADIANS + HOOD_ANGLE_OFFSET,
            Constants.Shooter.HOOD_MAX_POSITION_RADIANS + HOOD_ANGLE_OFFSET,
            true,
            Constants.Shooter.HOOD_STARTING_POSITION_RADIANS + HOOD_ANGLE_OFFSET,
            0.0,
            0.0);

    private final DCMotorSim flywheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotor, 0.0028087817, Constants.Shooter.FLYWHEEL_GEAR_RATIO),
            flywheelMotor,
            0.0,
            0.0);

    public ShooterSim(double periodicDt) {
        this.periodicDt = periodicDt;

        flywheelLeader = new TalonFX(RobotMap.Shooter.FLYWHEEL_LEADER_ID);
        flywheelFollower = new TalonFX(RobotMap.Shooter.FLYWHEEL_FOLLOWER_ID);
        hood = new TalonFX(RobotMap.Shooter.HOOD_ID);
        flywheelSimLeader = flywheelLeader.getSimState();
        flywheelSimFollower = flywheelFollower.getSimState();
        hoodSim = hood.getSimState();

        flywheelSimLeader.Orientation = ChassisReference.CounterClockwise_Positive;
        flywheelSimFollower.Orientation = ChassisReference.Clockwise_Positive;
        hoodSim.Orientation = ChassisReference.Clockwise_Positive;

        flywheelSimLeader.setMotorType(MotorType.KrakenX60);
        flywheelSimFollower.setMotorType(MotorType.KrakenX60);
        hoodSim.setMotorType(MotorType.KrakenX44);
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
        // Reset internal sim state
        flywheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        flywheelSimLeader.setRawRotorPosition(
                Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
        flywheelSimLeader.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
        flywheelSimLeader.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));

        flywheelSimFollower.setRawRotorPosition(
                Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
        flywheelSimFollower.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
        flywheelSimFollower.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));

        // Update internal raw position offset
        flywheelLeader.setPosition(newValue);
        flywheelFollower.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        // Reset internal sim state
        hoodSimModel.setState(Units.rotationsToRadians(newValueRotations) + HOOD_ANGLE_OFFSET, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        hoodSim.setRawRotorPosition(Constants.Shooter.HOOD_GEAR_RATIO
                * Units.radiansToRotations(hoodSimModel.getAngleRads()
                        - 2 * HOOD_ANGLE_OFFSET
                        - Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
        hoodSim.setRotorVelocity(
                Constants.Shooter.HOOD_GEAR_RATIO * Units.radiansToRotations(hoodSimModel.getVelocityRadPerSec()));

        // Update internal raw position offset
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
        return hoodSimModel.getCurrentDrawAmps();
    }

    @Override
    public void close() {
        flywheelLeader.close();
        flywheelFollower.close();
        hood.close();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
    }

    private void updateMotorSimulations() {
        flywheelSimLeader.setSupplyVoltage(RobotController.getBatteryVoltage());
        flywheelSimFollower.setSupplyVoltage(RobotController.getBatteryVoltage());
        hoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double hoodVoltage = hoodSim.getMotorVoltage();
        double flywheelLeaderVoltage = flywheelSimLeader.getMotorVoltage();
        double flywheelFollowerVoltage = flywheelSimFollower.getMotorVoltage();

        hoodSimModel.setInputVoltage(hoodVoltage);
        hoodSimModel.update(periodicDt);

        flywheelSimModel.setInputVoltage((flywheelLeaderVoltage + flywheelFollowerVoltage) / 2.0);
        flywheelSimModel.update(periodicDt);

        hoodSim.setRawRotorPosition(Constants.Shooter.HOOD_GEAR_RATIO
                * Units.radiansToRotations(hoodSimModel.getAngleRads()
                        - 2 * HOOD_ANGLE_OFFSET
                        - Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
        hoodSim.setRotorVelocity(
                Constants.Shooter.HOOD_GEAR_RATIO * Units.radiansToRotations(hoodSimModel.getVelocityRadPerSec()));

        flywheelSimLeader.setRawRotorPosition(
                Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
        flywheelSimLeader.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
        flywheelSimLeader.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));

        flywheelSimFollower.setRawRotorPosition(
                Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
        flywheelSimFollower.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
        flywheelSimFollower.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));
    }
}
