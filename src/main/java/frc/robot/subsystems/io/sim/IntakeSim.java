package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.RobotModel.FuelManager;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import frc.robot.utils.TalonFXOrchestra;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeSim implements IntakeIO {

    private static final double FUEL_VOLTAGE_MULTIPLIER_A = 16.73759;
    private static final double FUEL_VOLTAGE_MULTIPLIER_B = 4.1844;
    private static final double FUEL_VOLTAGE_MULTIPLIER_DIV = 4.0;

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);
    private static final Translation3d ROLLER_ORIGIN = new Translation3d(-0.594994, 0.329377, 0.163443);

    private static final double EJECT_BPS = 4.5;

    private final double periodicDt;

    private final TalonFXMotorGroup armGroup;
    private final TalonFX wheel;

    private final DCMotor armMotor = DCMotor.getKrakenX44(2);
    private final DCMotor wheelMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim armSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.2943129061, Constants.Intake.ARM_GEAR_RATIO),
            armMotor,
            Constants.Intake.ARM_GEAR_RATIO,
            0.253046091813, // distance from axis of rotation to center of mass
            Constants.Intake.ARM_DOWN_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            Constants.Intake.ARM_MAX_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            true,
            Constants.Intake.ARM_STARTING_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            0.0,
            0.0);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(wheelMotor, 0.0000819391, Constants.Intake.WHEEL_GEAR_RATIO),
            wheelMotor,
            0.0,
            0.0);

    private final IntakeSimulation intakeSimulation;

    private double lastEjectTime = 0.0;

    private AtomicInteger rollerFuelCount = new AtomicInteger(0);

    public IntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;

        armGroup = new TalonFXMotorGroup(
                "Intake Arm",
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));
        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);

        armGroup.getSimState(0).Orientation =
                ChassisReference.CounterClockwise_Positive; // Left of intake, right of robot // Arm up is positive
        armGroup.getSimState(1).Orientation =
                ChassisReference.Clockwise_Positive; // Right of intake, left of robot // Arm up is positive

        armGroup.getSimState(0).setMotorType(MotorType.KrakenX44);
        armGroup.getSimState(1).setMotorType(MotorType.KrakenX44);

        wheel.getSimState().Orientation = ChassisReference.Clockwise_Positive; // Positive is intake, negative is eject
        wheel.getSimState().setMotorType(MotorType.KrakenX44);

        RobotContainer.orchestra.add(wheel, TalonFXOrchestra.Tracks.INTAKE_WHEEL);
        RobotContainer.orchestra.add(armGroup.getMotor(0), TalonFXOrchestra.Tracks.INTAKE_ARM_LEFT);
        RobotContainer.orchestra.add(armGroup.getMotor(1), TalonFXOrchestra.Tracks.INTAKE_ARM_RIGHT);

        Rectangle intakeRect = IntakeSimulationUtils.getIntakeRectangle(
                drivetrainSim, WIDTH.in(Meters), LENGTH_EXTENDED.in(Meters), IntakeSimulation.IntakeSide.BACK);

        intakeSimulation = new IntakeSimulation("Fuel", drivetrainSim, intakeRect, FuelManager.getNodeCount());
        intakeSimulation.setCustomIntakeCondition(gm -> {
            boolean passed = wheel.getVelocity().getValueAsDouble() > Constants.Intake.WHEEL_INTAKE_VELOCITY_MPS - 2.0;
            if (passed) {
                try {
                    RobotContainer.model.fuelManager.intakeFuel(gm.getPose3d(), fuel -> {
                        if (new Translation2d(
                                                fuel.getPose().getX(),
                                                fuel.getPose().getZ())
                                        .getDistance(new Translation2d(ROLLER_ORIGIN.getX(), ROLLER_ORIGIN.getZ()))
                                > Constants.Intake.ROLLER_DIAMETER.in(Meters) / 2.0
                                        + Constants.Game.FUEL_DIAMETER_METERS / 2.0
                                        + 0.05) {
                            rollerFuelCount.decrementAndGet();
                            return true;
                        }
                        return false;
                    });
                    rollerFuelCount.incrementAndGet();
                } catch (IllegalStateException e) {
                    return false; // no available intake nodes, so fail the intake condition instead of crashing
                }
            }
            return passed;
        });

        RobotContainer.pdp.registerSimDevice(9, () -> armGroup.getSimState(0).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(10, () -> armGroup.getSimState(1).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(18, () -> wheel.getSupplyCurrent().getValue());
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        armGroup.applyConfig(configuration);
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheel.getConfigurator().apply(configuration);
    }

    @Override
    public void setArmControl(ControlRequest request) {
        armGroup.setControl(request);
    }

    @Override
    public void setWheelControl(ControlRequest request) {
        wheel.setControl(request);
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        // Reset internal sim state
        armSimModel.setState(
                Units.rotationsToRadians(newValue)
                        + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
                0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateArmRotor();

        // Update internal raw position offset
        armGroup.setPosition(newValue);
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        wheel.setPosition(newValue);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        armGroup.periodic();

        inputs.armHasPosition = !armGroup.hasLostPosition();
        inputs.armPositionRotations = armGroup.getAveragePosition();
        inputs.armVelocityRotationsPerSecond =
                SimpleMath.average(armGroup.getVelocities()).orElse(0);
        inputs.armVoltage = SimpleMath.average(armGroup.getVoltages()).orElse(0);
        inputs.armCurrentDraw = Arrays.stream(armGroup.getSimStates())
                .map(TalonFXSimState::getSupplyCurrentMeasure)
                .reduce(Amps.zero(), Current::plus);

        inputs.wheelConnected = wheel.isConnected();
        inputs.wheelPositionMeters = wheel.getPosition().getValueAsDouble();
        inputs.wheelVelocityMps = wheel.getVelocity().getValueAsDouble();
        inputs.wheelVoltage = wheel.getMotorVoltage().getValueAsDouble();
        inputs.wheelCurrentDraw = wheel.getSimState().getSupplyCurrentMeasure();
    }

    @Override
    public void close() {
        wheel.close();
        armGroup.close();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
        handleIntakeSimulation();

        Logger.recordOutput("Intake/SimCount", rollerFuelCount.get());
    }

    private void updateArmRotor() {
        for (TalonFXSimState simState : armGroup.getSimStates()) {
            simState.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                    * Units.radiansToRotations(armSimModel.getAngleRads()
                            - Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS)
                            - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
            simState.setRotorVelocity(
                    Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
        }
    }

    private void updateMotorSimulations() {
        for (TalonFXSimState simState : armGroup.getSimStates()) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }

        wheel.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double wheelVoltage = wheel.getSimState().getMotorVoltage();
        double armVoltage = SimpleMath.average(Arrays.stream(armGroup.getSimStates())
                        .mapToDouble(TalonFXSimState::getMotorVoltage)
                        .toArray())
                .orElse(0);

        wheelSimModel.setInputVoltage(wheelVoltage * calculateVoltageMultiplier(rollerFuelCount.get()));
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage(armVoltage);
        armSimModel.update(periodicDt);

        updateArmRotor();

        wheel.getSimState()
                .setRawRotorPosition(Constants.Intake.WHEEL_GEAR_RATIO * wheelSimModel.getAngularPositionRotations());
        wheel.getSimState()
                .setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                        * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));
        wheel.getSimState()
                .setRotorAcceleration(Constants.Intake.WHEEL_GEAR_RATIO
                        * Units.radiansToRotations(wheelSimModel.getAngularAccelerationRadPerSecSq()));
    }

    public boolean isExtendingHopperSpaceAvailable() {
        return Units.rotationsToRadians(armGroup.getAveragePosition())
                < Constants.Intake.ARM_DOWN_POSITION_RADIANS + Units.degreesToRadians(5.0);
    }

    private void handleIntakeSimulation() {
        if (Units.rotationsToRadians(armGroup.getAveragePosition())
                < Constants.Intake.ARM_DOWN_POSITION_RADIANS + Units.degreesToRadians(5.0)) {
            intakeSimulation.startIntake();
            if (wheel.getVelocity().getValueAsDouble() < Constants.Intake.WHEEL_EJECT_VELOCITY_MPS + 2.0) {
                double timeSinceLastEject = Timer.getTimestamp() - lastEjectTime;
                if (timeSinceLastEject > 1.0 / EJECT_BPS) {
                    RobotContainer.model.fuelManager.ejectFuel().ifPresent(fuel -> {
                        rollerFuelCount.incrementAndGet();
                        fuel.rotateAround(
                                () -> ROLLER_ORIGIN,
                                () -> new Translation3d(0, 1, 0),
                                new Translation2d(
                                                fuel.getPose().getX(),
                                                fuel.getPose().getZ())
                                        .getDistance(new Translation2d(ROLLER_ORIGIN.getX(), ROLLER_ORIGIN.getZ())),
                                () -> Math.PI / 2,
                                () -> -wheelSimModel.getAngularVelocityRadPerSec(),
                                () -> {
                                    rollerFuelCount.decrementAndGet();
                                    RobotContainer.model.fuelManager.toProjectile(
                                            fuel, RobotContainer.drivetrain.getSwerveDriveSimulation(), null);
                                });
                    });
                    lastEjectTime = Timer.getTimestamp();
                }
            }
        } else {
            intakeSimulation.stopIntake();
        }

        intakeSimulation.setGamePiecesCount(RobotContainer.model.fuelManager.getFuelCount());
    }

    private static double calculateVoltageMultiplier(int fuelCount) {
        return FUEL_VOLTAGE_MULTIPLIER_A / (fuelCount + FUEL_VOLTAGE_MULTIPLIER_B) / FUEL_VOLTAGE_MULTIPLIER_DIV;
    }
}
