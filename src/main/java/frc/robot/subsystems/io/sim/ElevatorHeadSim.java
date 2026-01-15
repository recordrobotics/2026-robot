package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.io.ElevatorHeadIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.maplesim.IntakeFromGoalSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ElevatorHeadSim implements ElevatorHeadIO {

    private static final double ALGAE_VOLTAGE_DIVIDER = 20.0;
    private static final double SHOOT_ALGAE_VELOCITY_THRESHOLD = 0.12; // m/s
    private static final double INTAKE_ALGAE_VELOCITY_THRESHOLD = 3; // m/s
    private static final double SHOOT_CORAL_VELOCITY_THRESHOLD = 1.4; // m/s
    private static final double CORAL_PROJECTILE_INITIAL_VELOCITY_MULTIPLIER = 2.0;
    private static final double ALGAE_PROJECTILE_INITIAL_VELOCITY_MULTIPLIER = 8.0;

    private static final int SHOOTER_CORAL_MOVEMENT_VELOCITY_FILTER_SIZE = 10;

    private final double periodicDt;

    private final SparkMax motor;
    private final SparkMaxSim motorSim;

    private boolean hadAlgae = false;

    private MedianFilter velocityFilter = new MedianFilter(SHOOTER_CORAL_MOVEMENT_VELOCITY_FILTER_SIZE);

    private final DCMotor wheelMotor = DCMotor.getNeo550(1);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(wheelMotor, 0.001, Constants.ElevatorHead.GEAR_RATIO), wheelMotor);

    private final DigitalInput coralDetector = new DigitalInput(RobotMap.ElevatorHead.PHOTOSENSOR_ID);
    private final SimDevice coralDetectorSim = SimDevice.create("DigitalInput", RobotMap.ElevatorHead.PHOTOSENSOR_ID);
    private final SimBoolean coralDetectorSimValue;

    private final AbstractDriveTrainSimulation drivetrainSim;

    private final IntakeFromGoalSimulation intakeSimulation;

    public ElevatorHeadSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;
        this.drivetrainSim = drivetrainSim;

        motor = new SparkMax(RobotMap.ElevatorHead.MOTOR_ID, MotorType.kBrushless);
        motorSim = new SparkMaxSim(motor, wheelMotor);

        if (coralDetectorSim != null)
            coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
        else coralDetectorSimValue = null;

        if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
        else coralDetector.close();

        intakeSimulation = new IntakeFromGoalSimulation("Algae", drivetrainSim, 1);
    }

    @Override
    public void setVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }

    @Override
    public void setPosition(double newValue) {
        motor.getEncoder().setPosition(newValue);
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
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
    public boolean isCoralDetectorTriggered() {
        if (coralDetectorSim != null) return coralDetectorSimValue.get();
        else return false;
    }

    public void setCoralDetectorSim(boolean newValue) {
        if (coralDetectorSimValue != null) coralDetectorSimValue.set(newValue);
    }

    @Override
    public double getCurrentDrawAmps() {
        return wheelSimModel.getCurrentDrawAmps();
    }

    public void setPreload() {
        setCoralDetectorSim(false);
        RobotContainer.model
                .getRobotCoral()
                .setPoseSupplier(RobotContainer.model.elevatorArm::getCoralShooterTargetPose);
    }

    public void clearPreload() {
        setCoralDetectorSim(true);
        RobotContainer.model.getRobotCoral().setPoseSupplier(() -> null);
    }

    @Override
    public void close() throws Exception {
        motor.close();
        if (coralDetectorSim != null) {
            coralDetectorSim.close();
            coralDetector.close();
        }
    }

    @Override
    public void simulationPeriodic() {
        double voltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();

        Pose3d algaeTargetPose = RobotContainer.model
                .elevatorArm
                .getAlgaeGrabberTargetPoseTop()
                .relativeTo(new Pose3d(RobotContainer.model.getRobot()));

        intakeSimulation.drivetrainRelativePose =
                new Transform3d(algaeTargetPose.getTranslation(), algaeTargetPose.getRotation());

        if (intakeSimulation.getGamePiecesAmount() > 0) {
            voltage /= ALGAE_VOLTAGE_DIVIDER;
            if (!hadAlgae) {
                RobotContainer.model
                        .getRobotAlgae()
                        .setPoseSupplier(RobotContainer.model.elevatorArm::getAlgaeGrabberTargetPoseTop);
                hadAlgae = true;
            }
        } else {
            hadAlgae = false;
        }

        wheelSimModel.setInputVoltage(voltage);
        wheelSimModel.update(periodicDt);

        motorSim.iterate(
                Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
                        * SimpleMath.SECONDS_PER_MINUTE
                        * Constants.ElevatorHead.GEAR_RATIO,
                RobotController.getBatteryVoltage(),
                periodicDt);

        velocityFilter.calculate(RobotContainer.elevatorHead.getVelocity());

        if (velocityFilter.lastValue() < -INTAKE_ALGAE_VELOCITY_THRESHOLD) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }

        if (!isCoralDetectorTriggered()
                && Math.abs(velocityFilter.lastValue()) > SHOOT_CORAL_VELOCITY_THRESHOLD
                && RobotContainer.elevatorHead.getCurrentCoralShooterState() != CoralShooterStates.POSITION) {
            RobotContainer.model.getRobotCoral().setPoseSupplier(() -> null);
            setCoralDetectorSim(true); // NC

            Pose3d ejectPose = RobotContainer.model
                    .elevatorArm
                    .getCoralShooterTargetPose()
                    .relativeTo(new Pose3d(RobotContainer.model.getRobot()));

            Angle angle = ejectPose.getRotation().getMeasureY();
            if (RobotContainer.elevatorHead.getVelocity() > 0) {
                angle = angle.unaryMinus();
            }

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            drivetrainSim.getSimulatedDriveTrainPose().getTranslation(),
                            ejectPose.toPose2d().getTranslation(),
                            // Obtain robot speed from drive simulation
                            drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            drivetrainSim.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            ejectPose.getMeasureZ(),
                            // The initial speed of the coral
                            MetersPerSecond.of(Math.abs(RobotContainer.elevatorHead.getVelocity())
                                    * CORAL_PROJECTILE_INITIAL_VELOCITY_MULTIPLIER /* help maplesim reef simulation */),
                            angle));
        }

        if (velocityFilter.lastValue() > SHOOT_ALGAE_VELOCITY_THRESHOLD
                && intakeSimulation.obtainGamePieceFromIntake()) {
            RobotContainer.model.getRobotAlgae().setPoseSupplier(() -> null);

            Angle angle = algaeTargetPose.getRotation().getMeasureY();

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                            // Obtain robot position from drive simulation
                            drivetrainSim.getSimulatedDriveTrainPose().getTranslation(),
                            algaeTargetPose.toPose2d().getTranslation(),
                            // Obtain robot speed from drive simulation
                            drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            drivetrainSim.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the algae is ejected
                            algaeTargetPose.getMeasureZ(),
                            // The initial speed of the algae
                            MetersPerSecond.of(Math.abs(RobotContainer.elevatorHead.getVelocity())
                                    * ALGAE_PROJECTILE_INITIAL_VELOCITY_MULTIPLIER /* help maplesim reef simulation */),
                            angle));
        }
    }
}
