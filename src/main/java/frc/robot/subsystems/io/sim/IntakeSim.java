package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeSim implements IntakeIO {

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);
    private static final Translation3d ROLLER_ORIGIN = new Translation3d(-0.594994, 0.329377, 0.163443);

    private static final double EJECT_BPS = 4.5;

    private final double periodicDt;

    private final TalonFX armLeader;
    private final TalonFX armFollower;
    private final TalonFX wheel;

    private final TalonFXSimState armSimLeader;
    private final TalonFXSimState armSimFollower;
    private final TalonFXSimState wheelSim;

    private final DCMotor armMotor = DCMotor.getKrakenX44(2);
    private final DCMotor wheelMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim armSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.2943129061, Constants.Intake.ARM_GEAR_RATIO),
            armMotor,
            Constants.Intake.ARM_GEAR_RATIO,
            0.253046091813, // distance from axis of rotation to center of mass
            Constants.Intake.ARM_DOWN_POSITION_RADIANS + Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS,
            Constants.Intake.ARM_MAX_POSITION_RADIANS + Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS,
            true,
            Constants.Intake.ARM_STARTING_POSITION_RADIANS + Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS,
            0.0,
            0.0);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(wheelMotor, 0.0000819391, Constants.Intake.WHEEL_GEAR_RATIO),
            wheelMotor,
            0.0,
            0.0);

    private final IntakeSimulation intakeSimulation;

    private double lastEjectTime = 0.0;

    public IntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;

        armLeader = new TalonFX(RobotMap.Intake.ARM_LEADER_ID);
        armFollower = new TalonFX(RobotMap.Intake.ARM_FOLLOWER_ID);
        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);
        armSimLeader = armLeader.getSimState();
        armSimFollower = armFollower.getSimState();
        wheelSim = wheel.getSimState();

        armSimLeader.Orientation =
                ChassisReference.CounterClockwise_Positive; // Left of intake, right of robot // Arm up is positive
        armSimFollower.Orientation =
                ChassisReference.Clockwise_Positive; // Right of intake, left of robot // Arm up is positive
        wheelSim.Orientation = ChassisReference.Clockwise_Positive; // Positive is intake, negative is eject

        armSimLeader.setMotorType(MotorType.KrakenX44);
        armSimFollower.setMotorType(MotorType.KrakenX44);
        wheelSim.setMotorType(MotorType.KrakenX44);

        Rectangle intakeRect = IntakeSimulationUtils.getIntakeRectangle(
                drivetrainSim, WIDTH.in(Meters), LENGTH_EXTENDED.in(Meters), IntakeSimulation.IntakeSide.BACK);

        intakeSimulation = new IntakeSimulation("Fuel", drivetrainSim, intakeRect, FuelManager.getNodeCount());
        intakeSimulation.setCustomIntakeCondition(gm -> {
            boolean passed = getWheelVelocityMps() > Constants.Intake.WHEEL_INTAKE_VELOCITY_MPS - 2.0;
            if (passed) {
                try {
                    RobotContainer.model.fuelManager.intakeFuel(gm.getPose3d());
                } catch (IllegalStateException e) {
                    return false; // no available intake nodes, so fail the intake condition instead of crashing
                }
            }
            return passed;
        });
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    @Override
    public Follower createArmFollower() {
        return new Follower(
                RobotMap.Intake.ARM_LEADER_ID, MotorAlignmentValue.Opposed); // motors face in opposite directions
    }

    @Override
    public void applyArmLeaderTalonFXConfig(TalonFXConfiguration configuration) {
        armLeader.getConfigurator().apply(configuration);
    }

    @Override
    public void applyArmFollowerTalonFXConfig(TalonFXConfiguration configuration) {
        armFollower.getConfigurator().apply(configuration);
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheel.getConfigurator().apply(configuration);
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        wheel.setVoltage(outputVolts);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        armLeader.setVoltage(outputVolts);
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        // Reset internal sim state
        wheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        wheelSim.setRawRotorPosition(Constants.Intake.WHEEL_GEAR_RATIO * wheelSimModel.getAngularPositionRotations());
        wheelSim.setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));
        wheelSim.setRotorAcceleration(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularAccelerationRadPerSecSq()));

        // Update internal raw position offset
        wheel.setPosition(newValue);
    }

    @Override
    public void setArmPositionRotations(double newValueRotations) {
        // Reset internal sim state
        armSimModel.setState(
                Units.rotationsToRadians(newValueRotations) + Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        armSimLeader.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads()
                        - Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS
                        - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimLeader.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        armSimFollower.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads()
                        - Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS
                        - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimFollower.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        // Update internal raw position offset
        armLeader.setPosition(newValueRotations);
        armFollower.setPosition(newValueRotations);
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
        wheel.setControl(request);
    }

    @Override
    public double getWheelPositionMeters() {
        return wheel.getPosition().getValueAsDouble();
    }

    @Override
    public double getWheelVelocityMps() {
        return wheel.getVelocity().getValueAsDouble();
    }

    @Override
    public double getWheelVoltage() {
        return wheel.getMotorVoltage().getValueAsDouble();
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
    public double getArmLeaderPositionRotations() {
        return armLeader.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmFollowerPositionRotations() {
        return armFollower.getPosition().getValueAsDouble();
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
        return wheelSimModel.getCurrentDrawAmps();
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
    public void close() {
        wheel.close();
        armLeader.close();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
        handleIntakeSimulation();
    }

    private void updateMotorSimulations() {
        armSimLeader.setSupplyVoltage(RobotController.getBatteryVoltage());
        armSimFollower.setSupplyVoltage(RobotController.getBatteryVoltage());
        wheelSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double wheelVoltage = wheelSim.getMotorVoltage();
        double armLeaderVoltage = armSimLeader.getMotorVoltage();
        double armFollowerVoltage = armSimFollower.getMotorVoltage();

        wheelSimModel.setInputVoltage(wheelVoltage);
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage((armLeaderVoltage + armFollowerVoltage) / 2.0);
        armSimModel.update(periodicDt);

        armSimLeader.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads()
                        - Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS
                        - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimLeader.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        armSimFollower.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads()
                        - Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS
                        - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimFollower.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        wheelSim.setRawRotorPosition(Constants.Intake.WHEEL_GEAR_RATIO * wheelSimModel.getAngularPositionRotations());
        wheelSim.setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));
        wheelSim.setRotorAcceleration(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularAccelerationRadPerSecSq()));
    }

    private void handleIntakeSimulation() {
        if (Units.rotationsToRadians(RobotContainer.intake.getArmPositionRotations())
                < Constants.Intake.ARM_DOWN_POSITION_RADIANS + Units.degreesToRadians(5.0)) {
            intakeSimulation.startIntake();
            if (getWheelVelocityMps() < Constants.Intake.WHEEL_EJECT_VELOCITY_MPS + 2.0) {
                double timeSinceLastEject = Timer.getTimestamp() - lastEjectTime;
                if (timeSinceLastEject > 1.0 / EJECT_BPS) {
                    RobotContainer.model.fuelManager.ejectFuel().ifPresent(fuel -> {
                        fuel.rotateAround(
                                () -> ROLLER_ORIGIN,
                                () -> new Translation3d(0, 1, 0),
                                new Translation2d(
                                                fuel.getPose().getX(),
                                                fuel.getPose().getZ())
                                        .getDistance(new Translation2d(ROLLER_ORIGIN.getX(), ROLLER_ORIGIN.getZ())),
                                () -> Math.PI / 2,
                                () -> -wheelSimModel.getAngularVelocityRadPerSec(),
                                () -> RobotContainer.model.fuelManager.toProjectile(
                                        fuel, RobotContainer.drivetrain.getSwerveDriveSimulation(), null));
                    });
                    lastEjectTime = Timer.getTimestamp();
                }
            }
        } else {
            intakeSimulation.stopIntake();
        }

        intakeSimulation.setGamePiecesCount(RobotContainer.model.fuelManager.getFuelCount());
    }
}
