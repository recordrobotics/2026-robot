package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.io.CoralIntakeIO;
import frc.robot.utils.DCMotors;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.ProjectileSimulationUtils;
import frc.robot.utils.SimpleMath;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class CoralIntakeSim implements CoralIntakeIO {

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);

    private static final double INTAKE_ANGLE_THRESHOLD = Units.degreesToRadians(30); // TODO check with real-world tests

    private static final double INTAKE_VELOCITY_THRESHOLD = 1.0;
    private static final double INTAKE_ARM_MAX_ANGLE_FOR_PROJECTILE_EJECT = Units.degreesToRadians(70);
    private static final double CORAL_PROJECTILE_EJECT_SPEED = 2.0;
    private static final double CORAL_GROUND_TOUCH_HEIGHT = 0.2;

    private final double periodicDt;

    private final SparkMax wheel;
    private final TalonFX arm;

    private final SparkMaxSim wheelSim;
    private final TalonFXSimState armSim;

    private final DCMotor wheelMotor = DCMotor.getNeo550(1);
    private final DCMotor armMotor = DCMotors.getKrakenX44(1);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Constants.CoralIntake.WHEEL_KV, Constants.CoralIntake.WHEEL_KA),
            wheelMotor,
            0.01,
            0.01);

    private final SingleJointedArmSim armSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.7, Constants.CoralIntake.ARM_GEAR_RATIO),
            armMotor,
            Constants.CoralIntake.ARM_GEAR_RATIO,
            Units.inchesToMeters(17.02),
            -0.95,
            Math.PI / 2,
            true,
            Constants.CoralIntake.ARM_START_POS,
            0.001,
            0.001);

    private final AbstractDriveTrainSimulation drivetrainSim;
    private final IntakeSimulation intakeSimulation;

    private boolean hadGamePieces = false;

    public CoralIntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;
        this.drivetrainSim = drivetrainSim;

        wheel = new SparkMax(RobotMap.CoralIntake.WHEEL_ID, MotorType.kBrushless);
        arm = new TalonFX(RobotMap.CoralIntake.ARM_ID);
        wheelSim = new SparkMaxSim(wheel, wheelMotor);
        armSim = arm.getSimState();

        Rectangle intakeRect = IntakeSimulationUtils.getIntakeRectangle(
                drivetrainSim, WIDTH.in(Meters), LENGTH_EXTENDED.in(Meters), IntakeSimulation.IntakeSide.LEFT);
        intakeRect.translate(Constants.CoralIntake.INTAKE_X_OFFSET.in(Meters), 0);

        intakeSimulation = new IntakeSimulation("Coral", drivetrainSim, intakeRect, 1);
        intakeSimulation.setCustomIntakeCondition(gp -> {
            boolean isNotStack = !(gp instanceof ReefscapeCoralAlgaeStack);
            double coralPointing =
                    (gp.getPoseOnField().getRotation().getRadians() + Math.PI) % Math.PI; // coral is bidirectional
            double robotPointing = (RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getRotation()
                                    .getRadians()
                            + Math.PI)
                    % Math.PI; // intake is bidirectional
            boolean coralAlignedWithRobot =
                    SimpleMath.isWithinTolerance(coralPointing, robotPointing, INTAKE_ANGLE_THRESHOLD);
            return isNotStack && coralAlignedWithRobot;
        });
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        arm.getConfigurator().apply(configuration);
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        wheel.setVoltage(outputVolts);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        arm.setVoltage(outputVolts);
    }

    @Override
    public void setWheelPosition(double newValue) {
        // Reset internal sim state
        wheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        wheelSim.setPosition(Constants.CoralIntake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularPositionRad()));
        wheelSim.setVelocity(Constants.CoralIntake.WHEEL_GEAR_RATIO
                * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                * SimpleMath.SECONDS_PER_MINUTE);

        // Update internal raw position offset
        wheel.getEncoder().setPosition(newValue);
    }

    @Override
    public void setArmPosition(double newValue) {
        // Reset internal sim state
        armSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        armSim.setRawRotorPosition(Constants.CoralIntake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads() - Constants.CoralIntake.ARM_START_POS));
        armSim.setRotorVelocity(
                Constants.CoralIntake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));

        // Update internal raw position offset
        arm.setPosition(newValue);
    }

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {
        arm.setControl(request);
    }

    @Override
    public double getWheelPosition() {
        return wheel.getEncoder().getPosition();
    }

    @Override
    public double getWheelVelocity() {
        return wheel.getEncoder().getVelocity();
    }

    @Override
    public double getWheelVoltage() {
        return wheel.getAppliedOutput() * wheel.getBusVoltage();
    }

    @Override
    public double getArmVoltage() {
        return arm.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmVelocity() {
        return arm.getVelocity().getValueAsDouble();
    }

    @Override
    public void setWheelPercent(double newValue) {
        wheel.set(newValue);
    }

    @Override
    public void setArmPercent(double newValue) {
        arm.set(newValue);
    }

    @Override
    public double getWheelPercent() {
        return wheel.get();
    }

    @Override
    public double getArmPercent() {
        return arm.get();
    }

    @Override
    public double getWheelCurrentDrawAmps() {
        return wheelSimModel.getCurrentDrawAmps();
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return arm.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        wheel.close();
        arm.close();
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

        double wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();
        double armVoltage = armSim.getMotorVoltage();

        wheelSimModel.setInputVoltage(wheelVoltage);
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage(armVoltage);
        armSimModel.update(periodicDt);

        wheelSim.iterate(
                Constants.CoralIntake.WHEEL_GEAR_RATIO
                        * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                        * SimpleMath.SECONDS_PER_MINUTE,
                RobotController.getBatteryVoltage(),
                periodicDt);

        armSim.setRawRotorPosition(Constants.CoralIntake.ARM_GEAR_RATIO
                * Units.radiansToRotations(armSimModel.getAngleRads() - Constants.CoralIntake.ARM_START_POS));
        armSim.setRotorVelocity(
                Constants.CoralIntake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
    }

    private void handleIntakeSimulation() {
        double wheelVelocity = RobotContainer.coralIntake.getWheelVelocity();

        if (wheelVelocity < -INTAKE_VELOCITY_THRESHOLD) {
            intakeSimulation.startIntake();
        } else if (wheelVelocity > INTAKE_VELOCITY_THRESHOLD) {
            handleCoralEjection();
            intakeSimulation.stopIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    private void handleCoralEjection() {
        if (intakeSimulation.obtainGamePieceFromIntake()
                && RobotContainer.coralIntake.getArmAngle() < INTAKE_ARM_MAX_ANGLE_FOR_PROJECTILE_EJECT) {
            createCoralProjectile();
        }
    }

    private void createCoralProjectile() {
        Pose3d ejectPose = RobotContainer.model
                .coralIntake
                .getCoralTargetPose()
                .relativeTo(new Pose3d(RobotContainer.model.getRobot()));
        RobotContainer.model.getRobotCoral().setPoseSupplier(() -> null);

        Rotation2d rot = drivetrainSim.getSimulatedDriveTrainPose().getRotation();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new GamePieceProjectile(
                                ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                                drivetrainSim
                                        .getSimulatedDriveTrainPose()
                                        .getTranslation()
                                        .plus(ejectPose
                                                .toPose2d()
                                                .getTranslation()
                                                .rotateBy(rot)),
                                ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                                        ejectPose.toPose2d().getTranslation(),
                                        drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        rot.plus(Rotation2d.kCCW_90deg),
                                        CORAL_PROJECTILE_EJECT_SPEED
                                                * Math.cos(RobotContainer.coralIntake.getArmAngle())),
                                ejectPose.getZ(),
                                CORAL_PROJECTILE_EJECT_SPEED * Math.sin(RobotContainer.coralIntake.getArmAngle()),
                                RobotContainer.model
                                        .coralIntake
                                        .getCoralTargetPose()
                                        .getRotation())
                        .withTouchGroundHeight(CORAL_GROUND_TOUCH_HEIGHT)
                        .enableBecomesGamePieceOnFieldAfterTouchGround());
    }

    private void updateGamePieceState() {
        if (intakeSimulation.getGamePiecesAmount() != 0) {
            if (!hadGamePieces) {
                RobotContainer.model
                        .getRobotCoral()
                        .setPoseSupplier(RobotContainer.model.coralIntake::getCoralTargetPose);
                hadGamePieces = true;
            }
        } else {
            hadGamePieces = false;
        }
    }
}
