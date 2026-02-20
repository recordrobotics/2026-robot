package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private final SimDevice frontLeftLimitSwitchSim =
            SimDevice.create("DigitalInput", RobotMap.Turret.FRONT_LEFT_LIMIT_SWITCH_ID);
    private final SimDevice backLeftLimitSwitchSim =
            SimDevice.create("DigitalInput", RobotMap.Turret.BACK_LEFT_LIMIT_SWITCH_ID);
    private final SimDevice backRightLimitSwitchSim =
            SimDevice.create("DigitalInput", RobotMap.Turret.BACK_RIGHT_LIMIT_SWITCH_ID);

    private final SimBoolean frontLeftLimitSwitchSimValue;
    private final SimBoolean backLeftLimitSwitchSimValue;
    private final SimBoolean backRightLimitSwitchSimValue;

    public TurretSim(double periodicDt) {
        this.periodicDt = periodicDt;

        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
        turretSimState = turret.getSimState();

        turretSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        turretSimState.setMotorType(MotorType.KrakenX44);

        if (frontLeftLimitSwitchSim != null)
            frontLeftLimitSwitchSimValue = frontLeftLimitSwitchSim.createBoolean("Value", Direction.kOutput, true);
        else frontLeftLimitSwitchSimValue = null;

        if (frontLeftLimitSwitchSim != null) frontLeftLimitSwitch.setSimDevice(frontLeftLimitSwitchSim);
        else frontLeftLimitSwitch.close();

        if (backLeftLimitSwitchSim != null)
            backLeftLimitSwitchSimValue = backLeftLimitSwitchSim.createBoolean("Value", Direction.kOutput, true);
        else backLeftLimitSwitchSimValue = null;

        if (backLeftLimitSwitchSim != null) backLeftLimitSwitch.setSimDevice(backLeftLimitSwitchSim);
        else backLeftLimitSwitch.close();

        if (backRightLimitSwitchSim != null)
            backRightLimitSwitchSimValue = backRightLimitSwitchSim.createBoolean("Value", Direction.kOutput, true);
        else backRightLimitSwitchSimValue = null;

        if (backRightLimitSwitchSim != null) backRightLimitSwitch.setSimDevice(backRightLimitSwitchSim);
        else backRightLimitSwitch.close();
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

        updateLimitSwitches();
    }

    @Override
    public LimitSwitchStates getLimitSwitchStates() {
        return new LimitSwitchStates(
                frontLeftLimitSwitchSim != null && !frontLeftLimitSwitchSimValue.get(),
                backLeftLimitSwitchSim != null && !backLeftLimitSwitchSimValue.get(),
                backRightLimitSwitchSim != null && !backRightLimitSwitchSimValue.get());
    }

    @Override
    public void close() {
        turret.close();
        if (frontLeftLimitSwitchSim != null) {
            frontLeftLimitSwitchSim.close();
            frontLeftLimitSwitch.close();
        }
        if (backLeftLimitSwitchSim != null) {
            backLeftLimitSwitchSim.close();
            backLeftLimitSwitch.close();
        }
        if (backRightLimitSwitchSim != null) {
            backRightLimitSwitchSim.close();
            backRightLimitSwitch.close();
        }
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
        double turretPositionRad = turretSimModel.getAngularPositionRad();

        // Wrap turret position within one rotation
        turretPositionRad = MathUtil.inputModulus(
                turretPositionRad - Constants.Turret.TURRET_MAGNET_OFFSET_ANGLE_RAD, 0, Math.PI * 2);

        frontLeftLimitSwitchSimModel.update(turretPositionRad);
        backLeftLimitSwitchSimModel.update(turretPositionRad);
        backRightLimitSwitchSimModel.update(turretPositionRad);

        if (frontLeftLimitSwitchSimValue != null)
            frontLeftLimitSwitchSimValue.set(!frontLeftLimitSwitchSimModel.isTriggered());
        if (backLeftLimitSwitchSimValue != null)
            backLeftLimitSwitchSimValue.set(!backLeftLimitSwitchSimModel.isTriggered());
        if (backRightLimitSwitchSimValue != null)
            backRightLimitSwitchSimValue.set(!backRightLimitSwitchSimModel.isTriggered());
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
