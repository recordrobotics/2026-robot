package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Turret;
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

    private final DigitalInput frontLeftLimitSwitch = new DigitalInput(RobotMap.Turret.FRONT_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backLeftLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backRightLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_RIGHT_LIMIT_SWITCH_ID);

    private final MagneticLimitSwitch frontLeftLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.FRONT_LEFT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);
    private final MagneticLimitSwitch backLeftLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.BACK_LEFT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);
    private final MagneticLimitSwitch backRightLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.BACK_RIGHT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);

    private final DIOSim frontLeftLimitSwitchSim;
    private final DIOSim backLeftLimitSwitchSim;
    private final DIOSim backRightLimitSwitchSim;

    public TurretSim(double periodicDt) {
        this.periodicDt = periodicDt;

        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
        turretSimState = turret.getSimState();

        turretSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        turretSimState.setMotorType(MotorType.KrakenX44);

        frontLeftLimitSwitchSim = new DIOSim(frontLeftLimitSwitch);
        backLeftLimitSwitchSim = new DIOSim(backLeftLimitSwitch);
        backRightLimitSwitchSim = new DIOSim(backRightLimitSwitch);
        frontLeftLimitSwitchSim.setIsInput(true);
        backLeftLimitSwitchSim.setIsInput(true);
        backRightLimitSwitchSim.setIsInput(true);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        turret.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        turret.setControl(request);
    }

    @Override
    public boolean hasHitForwardSoftLimit() {
        return turret.getFault_ForwardSoftLimit().getValue().booleanValue();
    }

    @Override
    public boolean hasHitReverseSoftLimit() {
        return turret.getFault_ReverseSoftLimit().getValue().booleanValue();
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

        updateLimitSwitches();
    }

    @Override
    public LimitSwitchStates getLimitSwitchStates() {
        return new LimitSwitchStates(
                !frontLeftLimitSwitch.get(), !backLeftLimitSwitch.get(), !backRightLimitSwitch.get());
    }

    @Override
    public void close() {
        turret.close();
        frontLeftLimitSwitch.close();
        backLeftLimitSwitch.close();
        backRightLimitSwitch.close();
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

        updateLimitSwitches();
    }

    private void updateLimitSwitches() {
        double turretPositionRad = Units.rotationsToRadians(
                turretSimModel.getAngularPositionRotations() + Turret.MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);

        // Wrap turret position within one rotation
        turretPositionRad = MathUtil.inputModulus(
                turretPositionRad - Constants.Turret.TURRET_MAGNET_OFFSET_ANGLE_RAD, 0, Math.PI * 2);

        frontLeftLimitSwitchSimModel.update(turretPositionRad);
        backLeftLimitSwitchSimModel.update(turretPositionRad);
        backRightLimitSwitchSimModel.update(turretPositionRad);

        frontLeftLimitSwitchSim.setValue(!frontLeftLimitSwitchSimModel.isTriggered());
        backLeftLimitSwitchSim.setValue(!backLeftLimitSwitchSimModel.isTriggered());
        backRightLimitSwitchSim.setValue(!backRightLimitSwitchSimModel.isTriggered());
    }

    private static class MagneticLimitSwitch {
        private final double positionRad;
        private final double triggerAngleRad;
        private final double detriggerAngleRad;

        private boolean triggered = false;

        public MagneticLimitSwitch(double positionRad, double triggerAngleRad, double detriggerAngleRad) {
            this.positionRad = positionRad;
            this.triggerAngleRad = triggerAngleRad;
            this.detriggerAngleRad = detriggerAngleRad;
        }

        public boolean update(double turretPositionRad) {
            double angleDifference = Math.abs(turretPositionRad - positionRad);

            if (!triggered && angleDifference <= triggerAngleRad) {
                triggered = true;
            } else if (triggered && angleDifference >= detriggerAngleRad) {
                triggered = false;
            }

            return triggered;
        }

        public boolean isTriggered() {
            return triggered;
        }
    }
}
