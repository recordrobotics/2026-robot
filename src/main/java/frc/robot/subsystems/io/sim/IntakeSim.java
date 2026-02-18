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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.SimpleMath;
import java.util.Random;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class IntakeSim implements IntakeIO {

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);

    private static final Random RANDOM = new Random();

    private static final double CORAL_GROUND_TOUCH_HEIGHT = 0.1; // meters

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
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.7, Constants.Intake.ARM_GEAR_RATIO),
            armMotor,
            Constants.Intake.ARM_GEAR_RATIO,
            Units.inchesToMeters(17.02),
            Constants.Intake.ARM_DOWN_POSITION_RADIANS,
            Constants.Intake.ARM_STARTING_POSITION_RADIANS,
            true,
            Constants.Intake.ARM_STARTING_POSITION_RADIANS,
            0.0,
            0.0);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Constants.Intake.WHEEL_KV, Constants.Intake.WHEEL_KA),
            wheelMotor,
            0.0,
            0.0);

    private final IntakeSimulation intakeSimulation;

    private int fuelCount = 0;
    private double intakeTimeAccumulatorMillis = 0.0;

    private AbstractDriveTrainSimulation drivetrainSim;

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

        intakeSimulation =
                new IntakeSimulation("Fuel", drivetrainSim, intakeRect, Constants.Intake.MAX_INTAKE_CAPACITY);
        this.drivetrainSim = drivetrainSim;
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
    public void setWheelPositionMps(double newValue) {
        // Reset internal sim state
        wheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        wheelSim.setRawRotorPosition(
                Constants.Intake.WHEEL_GEAR_RATIO * Units.radiansToRotations(wheelSimModel.getAngularPositionRad()));
        wheelSim.setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                * SimpleMath.SECONDS_PER_MINUTE);

        // Update internal raw position offset
        wheel.setPosition(newValue);
    }

    @Override
    public void setArmPositionRotations(double newValueRotations) {
        // Reset internal sim state
        armSimModel.setState(Units.rotationsToRadians(newValueRotations), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        armSimLeader.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(
                        armSimModel.getAngleRads() - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimLeader.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        armSimFollower.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(
                        armSimModel.getAngleRads() - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
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
    public double getWheelPositionRotations() {
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

    public boolean addFuel() {
        return intakeSimulation.addGamePieceToIntake();
    }

    public boolean removeFuel() {
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    private static Pair<Translation3d, Translation3d> getEjectionData(
            AbstractDriveTrainSimulation drivetrainSimulation) {
        Pose3d basePose = new Pose3d(drivetrainSimulation.getSimulatedDriveTrainPose());
        Translation3d pose = basePose.transformBy(new Transform3d(
                        Meters.of(-0.5), Meters.of(-0.5 + RANDOM.nextDouble()), Meters.of(0.1), Rotation3d.kZero))
                .getTranslation();
        Translation3d velocity = basePose.transformBy(
                        new Transform3d(Meters.of(-1.0), Meters.of(0.0), Meters.of(0.0), Rotation3d.kZero))
                .getTranslation();
        return new Pair<>(pose, velocity);
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
        handleIntakeSimulation();
        updateGamePieceState();
    }

    private void updateMotorSimulations() {
        armSimLeader.setSupplyVoltage(RobotController.getBatteryVoltage());
        armSimFollower.setSupplyVoltage(RobotController.getBatteryVoltage());
        wheelSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double wheelVoltage =
                wheelSim.getMotorVoltage(); // TODO what is the difference between this and getWheelVoltage()?
        double armLeaderVoltage =
                armSimLeader.getMotorVoltage(); // TODO what is the difference between this and getArmVoltage()?
        double armFollowerVoltage = armSimFollower.getMotorVoltage();

        wheelSimModel.setInputVoltage(wheelVoltage);
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage((armLeaderVoltage + armFollowerVoltage) / 2.0);
        armSimModel.update(periodicDt);

        armSimLeader.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(
                        armSimModel.getAngleRads() - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimLeader.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        armSimFollower.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(
                        armSimModel.getAngleRads() - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armSimFollower.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        wheelSim.setRawRotorPosition(
                Constants.Intake.WHEEL_GEAR_RATIO * Units.radiansToRotations(wheelSimModel.getAngularPositionRad()));
        wheelSim.setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                * SimpleMath.SECONDS_PER_MINUTE);
    }

    private void handleIntakeSimulation() {
        if (RobotContainer.intake.getTargetState() == Intake.IntakeState.INTAKE
                && RobotContainer.intake.atGoal()
                && (fuelCount < Constants.Intake.MAX_INTAKE_CAPACITY
                        || /* TODO RobotContainer.hopper.getFuelCount() */ 0
                                < Constants.Hopper.MAX_HOPPER_CAPACITY)) { // intake or hopper has more room
            intakeSimulation.startIntake();
        } else if (RobotContainer.intake.getTargetState() == Intake.IntakeState.EJECT
                && RobotContainer.intake.atGoal()
                && fuelCount > 0) {
            // Limit fuel ejection rate to Constants.Intake.EJECT_FUEL_PER_SECOND
            intakeTimeAccumulatorMillis += periodicDt;
            while (intakeTimeAccumulatorMillis >= 1000.0 / Constants.Intake.EJECT_FUEL_PER_SECOND && fuelCount > 0) {
                createProjectile();
                intakeTimeAccumulatorMillis -= 1000.0 / Constants.Intake.EJECT_FUEL_PER_SECOND;
                fuelCount--;
            }
        } else {
            intakeSimulation.stopIntake();
        }
    }

    private void createProjectile() {
        Pair<Translation3d, Translation3d> ejectionData = getEjectionData(drivetrainSim);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new GamePieceProjectile(
                                RebuiltFuelOnField.REBUILT_FUEL_INFO,
                                ejectionData.getFirst().toTranslation2d(),
                                ejectionData.getSecond().toTranslation2d(),
                                ejectionData.getFirst().getZ(),
                                ejectionData.getSecond().getZ(),
                                Rotation3d.kZero)
                        .withTouchGroundHeight(CORAL_GROUND_TOUCH_HEIGHT)
                        .enableBecomesGamePieceOnFieldAfterTouchGround());
    }

    private void updateGamePieceState() {
        fuelCount += intakeSimulation.getGamePiecesAmount();
    }
}
