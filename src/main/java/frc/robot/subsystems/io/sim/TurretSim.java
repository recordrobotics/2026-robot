package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.TurretIO;

public class TurretSim implements TurretIO {

    private final double periodicDt;

    private final TalonFX turret;
    private final TalonFXSimState turretSimState;
    private final DCMotor turretMotor = DCMotor.getKrakenX44(1);

    private final DCMotorSim turretSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretMotor, 0.0962647197, Constants.Turret.GEAR_RATIO),
            turretMotor,
            0.0,
            0.0);

    public TurretSim(double periodicDt) {
        this.periodicDt = periodicDt;

        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
        turretSimState = turret.getSimState();

        turretSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        turretSimState.setMotorType(MotorType.KrakenX44);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        turret.getConfigurator().apply(configuration);
    }

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {
        turret.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        turret.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return turret.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return turret.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return turret.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return turretSimModel.getCurrentDrawAmps();
    }

    @Override
    public void setPositionRotations(double newValue) {
        // reset internal sim state
        turretSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        turretSimState.setRawRotorPosition(Constants.Turret.GEAR_RATIO * turretSimModel.getAngularPositionRotations());
        turretSimState.setRotorVelocity(
                Constants.Turret.GEAR_RATIO * Units.radiansToRotations(turretSimModel.getAngularVelocityRadPerSec()));
        turretSimState.setRotorAcceleration(Constants.Turret.GEAR_RATIO
                * Units.radiansToRotations(turretSimModel.getAngularAccelerationRadPerSecSq()));

        // Update internal raw position offset
        turret.setPosition(newValue);
    }

    @Override
    public LimitSwitchStates getLimitSwitchStates() {
        return LimitSwitchStates.NO_HITS;
    }

    @Override
    public void close() {
        turret.close();
    }

    @Override
    public void simulationPeriodic() {
        turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double turretVoltage = turretSimState.getMotorVoltage();

        turretSimModel.setInputVoltage(turretVoltage);
        turretSimModel.update(periodicDt);

        turretSimState.setRawRotorPosition(Constants.Turret.GEAR_RATIO * turretSimModel.getAngularPositionRotations());
        turretSimState.setRotorVelocity(
                Constants.Turret.GEAR_RATIO * Units.radiansToRotations(turretSimModel.getAngularVelocityRadPerSec()));
        turretSimState.setRotorAcceleration(Constants.Turret.GEAR_RATIO
                * Units.radiansToRotations(turretSimModel.getAngularAccelerationRadPerSecSq()));
    }
}
