package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.SimpleMath;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeSim implements IntakeIO {

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);

    private static final double INTAKE_VELOCITY_THRESHOLD = 1.0;

    private final double periodicDt;

    private final TalonFX wheel;
    private final TalonFX armLeader;
    private final TalonFX armFollower;

    private final TalonFXSimState wheelSim;
    private final TalonFXSimState armSim;

    private final DCMotor wheelMotor = DCMotor.getKrakenX44(1); // TODO is correct motor(s)?
    private final DCMotor armMotor = DCMotor.getKrakenX44(2); // TODO is correct motor(s)?

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Constants.Intake.WHEEL_KV, Constants.Intake.WHEEL_KA),
            wheelMotor,
            0.01,
            0.01);

    private final SingleJointedArmSim armSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.7, Constants.Intake.ARM_GEAR_RATIO),
            armMotor,
            Constants.Intake.ARM_GEAR_RATIO,
            Units.inchesToMeters(17.02),
            -0.95,
            Math.PI / 2,
            true,
            Constants.Intake.ARM_UP_POSITION,
            0.001,
            0.001);

    private final IntakeSimulation intakeSimulation;

    private boolean hadGamePieces = false;

    public IntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;

        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);
        armLeader = new TalonFX(RobotMap.Intake.ARM_LEADER_ID);
        armFollower = new TalonFX(RobotMap.Intake.ARM_FOLLOWER_ID);
        wheelSim = new TalonFXSimState(wheel);
        armSim = armLeader.getSimState();

        Rectangle intakeRect = IntakeSimulationUtils.getIntakeRectangle(
                drivetrainSim, WIDTH.in(Meters), LENGTH_EXTENDED.in(Meters), IntakeSimulation.IntakeSide.BACK);

        intakeSimulation = new IntakeSimulation("Fuel", drivetrainSim, intakeRect, Constants.Intake.ROBOT_MAX_CAPACITY);
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public Follower createArmFollower() {
        return new Follower(RobotMap.Intake.ARM_LEADER_ID, MotorAlignmentValue.Opposed); // motors face in opposite
        // directions
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        armLeader.getConfigurator().apply(configuration);
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
    public void setWheelPositionRotations(double newValue) {
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
    public void setArmPositionRotations(double newValue) {
        // Reset internal sim state
        armSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        armSim.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads() - Constants.Intake.ARM_UP_POSITION));
        armSim.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        // Update internal raw position offset
        armLeader.setPosition(newValue);
        armFollower.setPosition(newValue);
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
    public double getWheelVelocityRotationsPerSecond() {
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
    public void close() throws Exception {
        wheel.close();
        armLeader.close();
    }

    public void addCoral() {
        intakeSimulation.addGamePieceToIntake();
    }

    public void removeCoral() {
        intakeSimulation.obtainGamePieceFromIntake();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
        handleIntakeSimulation();
        updateGamePieceState();
    }

    private void updateMotorSimulations() {
        armSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        wheelSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double wheelVoltage =
                wheelSim.getMotorVoltage(); // TODO what is the difference between this and getWheelVoltage()?
        double armVoltage = armSim.getMotorVoltage(); // TODO what is the difference between this and getArmVoltage()?

        wheelSimModel.setInputVoltage(wheelVoltage);
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage(armVoltage);
        armSimModel.update(periodicDt);

        armSim.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads() - Constants.Intake.ARM_UP_POSITION));
        armSim.setRotorVelocity(
                Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        wheelSim.setRawRotorPosition(
                Constants.Intake.WHEEL_GEAR_RATIO * Units.radiansToRotations(wheelSimModel.getAngularPositionRad()));
        wheelSim.setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                * SimpleMath.SECONDS_PER_MINUTE);
    }

    private void handleIntakeSimulation() {
        double wheelVelocity = RobotContainer.intake.getWheelVelocityRotationsPerSecond();

        if (wheelVelocity < -INTAKE_VELOCITY_THRESHOLD) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    private void updateGamePieceState() {
        if (intakeSimulation.getGamePiecesAmount() != 0) {
            if (!hadGamePieces) { // just intaked a game piece
                // TODO handle transfering game piece from intake to hopper
                hadGamePieces = true;
            }
        } else {
            hadGamePieces = false;
        }
    }
}
