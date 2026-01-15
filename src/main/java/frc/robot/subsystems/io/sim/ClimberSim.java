package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberSim implements ClimberIO {

    private final double periodicDt;

    private final TalonFX motor;

    private final TalonFXSimState motorSim;

    private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

    private final Servo ratchet;
    private final SimDevice ratchetSim;
    private final SimDouble ratchetValue;

    private final SingleJointedArmSim simModel = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(Constants.Climber.KV, Constants.Climber.KA),
            dcMotor,
            Constants.Climber.GEAR_RATIO,
            Units.inchesToMeters(88),
            Units.degreesToRadians(-90),
            Units.degreesToRadians(150),
            true,
            Constants.Climber.START_ROTATIONS.in(Radians),
            0.001,
            0.001);

    @SuppressWarnings("java:S2583") // SimDevice.create can return null
    public ClimberSim(double periodicDt) {
        this.periodicDt = periodicDt;

        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
        motorSim = motor.getSimState();

        ratchet = new Servo(RobotMap.Climber.RATCHET_SERVO_ID);
        ratchetSim = SimDevice.create("Servo", RobotMap.Climber.RATCHET_SERVO_ID);

        if (ratchetSim != null) ratchetValue = ratchetSim.createDouble("Value", Direction.kOutput, 0);
        else ratchetValue = null;

        if (ratchetSim != null) DIOJNI.setDIOSimDevice(ratchet.getHandle(), ratchetSim.getNativeHandle());
        else ratchet.close();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setMotionMagic(MotionMagicVoltage request) {
        motor.setControl(request);
    }

    @Override
    public void setVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }

    @Override
    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setPercent(double newValue) {
        motor.set(newValue);
    }

    @Override
    public double getPercent() {
        return motor.get();
    }

    @Override
    public double getCurrentDrawAmps() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setRatchet(double value) {
        if (ratchetSim != null) {
            ratchetValue.set(value);
        }

        ratchet.set(value);
    }

    @Override
    public void close() throws Exception {
        motor.close();
        ratchet.close();
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double voltage = motorSim.getMotorVoltage();

        simModel.setInputVoltage(voltage);
        simModel.update(periodicDt);

        motorSim.setRawRotorPosition((simModel.getAngleRads() - Constants.Climber.START_ROTATIONS.in(Radians))
                * Constants.Climber.GEAR_RATIO);
        motorSim.setRotorVelocity(simModel.getVelocityRadPerSec() * Constants.Climber.GEAR_RATIO);
    }
}
