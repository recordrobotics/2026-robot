package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import java.util.Arrays;

public class ShooterSim implements ShooterIO {

    private static final double FLYWHEEL_SHOOT_VOLTAGE_MULTIPLIER = 0.88;

    private final double periodicDt;

    private final TalonFXMotorGroup flywheelGroup;
    private final TalonFX hood;

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(2);
    private final DCMotor hoodMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim hoodSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(hoodMotor, 0.007440336, Constants.Shooter.HOOD_GEAR_RATIO),
            hoodMotor,
            Constants.Shooter.HOOD_GEAR_RATIO,
            0.107361364429, // distance from hood pinion to center of mass
            Constants.Shooter.HOOD_MIN_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            Constants.Shooter.HOOD_MAX_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            true,
            Constants.Shooter.HOOD_STARTING_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            0.0,
            0.0);

    private final DCMotorSim flywheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotor, 0.0006979201, Constants.Shooter.FLYWHEEL_GEAR_RATIO),
            flywheelMotor,
            0.0,
            0.0);

    public ShooterSim(double periodicDt) {
        this.periodicDt = periodicDt;

        flywheelGroup = new TalonFXMotorGroup(
                "Shooter",
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));
        hood = new TalonFX(RobotMap.Shooter.HOOD_ID);

        flywheelGroup.getSimState(0).Orientation = ChassisReference.CounterClockwise_Positive;
        flywheelGroup.getSimState(1).Orientation = ChassisReference.Clockwise_Positive;
        hood.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        flywheelGroup.getSimState(0).setMotorType(MotorType.KrakenX60);
        flywheelGroup.getSimState(1).setMotorType(MotorType.KrakenX60);
        hood.getSimState().setMotorType(MotorType.KrakenX44);

        RobotContainer.pdp.registerSimDevice(
                15, () -> flywheelGroup.getSimState(0).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(
                16, () -> flywheelGroup.getSimState(1).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(17, () -> hood.getSimState().getSupplyCurrentMeasure());
    }

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelGroup.applyConfig(configuration);
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        hood.getConfigurator().apply(configuration);
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        // Reset internal sim state
        flywheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateFlywheelRotor();

        // Update internal raw position offset
        flywheelGroup.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        // Reset internal sim state
        hoodSimModel.setState(
                Units.rotationsToRadians(newValueRotations + Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
                0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateHoodRotor();

        // Update internal raw position offset
        hood.setPosition(newValueRotations);
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        flywheelGroup.setControl(request);
    }

    @Override
    public void setHoodControl(ControlRequest request) {
        hood.setControl(request);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelGroup.periodic();
        if (flywheelGroup.hasLostPosition()) { // position doesn't matter
            flywheelGroup.setPosition(0);
        }

        inputs.flywheelPositionMeters = flywheelGroup.getAveragePosition();
        inputs.flywheelVelocityMps = SimpleMath.average(flywheelGroup.getVelocities());
        inputs.flywheelVoltage = SimpleMath.average(flywheelGroup.getVoltages());
        inputs.flywheelCurrentDraw = Arrays.stream(flywheelGroup.getSimStates())
                .map(TalonFXSimState::getSupplyCurrentMeasure)
                .reduce(Amps.zero(), Current::plus);

        inputs.hoodConnected = hood.isConnected();
        inputs.hoodPositionRotations = hood.getPosition().getValueAsDouble();
        inputs.hoodVelocityRotationsPerSecond = hood.getVelocity().getValueAsDouble();
        inputs.hoodVoltage = hood.getMotorVoltage().getValueAsDouble();
        inputs.hoodCurrentDraw = hood.getSimState().getSupplyCurrentMeasure();
    }

    @Override
    public void close() {
        flywheelGroup.close();
        hood.close();
    }

    private void updateFlywheelRotor() {
        for (TalonFXSimState simState : flywheelGroup.getSimStates()) {
            simState.setRawRotorPosition(
                    Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
            simState.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                    * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
            simState.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                    * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));
        }
    }

    private void updateHoodRotor() {
        hood.getSimState()
                .setRawRotorPosition(Constants.Shooter.HOOD_GEAR_RATIO
                                * Units.radiansToRotations(
                                        hoodSimModel.getAngleRads() - Constants.Shooter.HOOD_STARTING_POSITION_RADIANS)
                        - Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS);
        hood.getSimState()
                .setRotorVelocity(Constants.Shooter.HOOD_GEAR_RATIO
                        * Units.radiansToRotations(hoodSimModel.getVelocityRadPerSec()));
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
    }

    private void updateMotorSimulations() {
        for (TalonFXSimState simState : flywheelGroup.getSimStates()) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }

        hood.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double hoodVoltage = hood.getSimState().getMotorVoltage();
        double flywheelVoltage = SimpleMath.average(Arrays.stream(flywheelGroup.getSimStates())
                .mapToDouble(TalonFXSimState::getMotorVoltage)
                .toArray());

        hoodSimModel.setInputVoltage(hoodVoltage);
        hoodSimModel.update(periodicDt);

        flywheelSimModel.setInputVoltage(
                (RobotContainer.model.fuelManager.isShootingFuel() ? FLYWHEEL_SHOOT_VOLTAGE_MULTIPLIER : 1.0)
                        * flywheelVoltage);
        flywheelSimModel.update(periodicDt);

        updateHoodRotor();
        updateFlywheelRotor();
    }
}
