package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXOrchestra;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class ClimberSim implements ClimberIO {

    private final double periodicDt;

    private final TalonFX motorClimber;

    private AbstractDriveTrainSimulation drivetrainSim;

    private final ElevatorSim physicsSimNotSupportingRobot = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Climber.GEAR_RATIO,
            Constants.Climber.CARRIAGE_MASS_KG,
            Constants.Climber.SPROCKET_EFFECTIVE_RADIUS,
            0,
            Constants.Climber.MAX_HEIGHT_METERS,
            true,
            0,
            0.0003,
            0.0003);

    private final ElevatorSim physicsSimSupportingRobot = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Climber.GEAR_RATIO,
            Constants.Frame.ROBOT_MASS_KG - Constants.Climber.CARRIAGE_MASS_KG,
            Constants.Climber.SPROCKET_EFFECTIVE_RADIUS,
            0,
            Constants.Climber.MAX_HEIGHT_METERS,
            true,
            0,
            0.0003,
            0.0003);

    private ElevatorSim currentPhysicsSim = physicsSimNotSupportingRobot;

    public ClimberSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;
        this.drivetrainSim = drivetrainSim;

        motorClimber = new TalonFX(RobotMap.Climber.MOTOR_ID);
        RobotContainer.orchestra.add(motorClimber, TalonFXOrchestra.Tracks.CLIMBER);

        motorClimber.getSimState().Orientation = ChassisReference.Clockwise_Positive; // correct

        RobotContainer.pdp.registerSimDevice(12, motorClimber.getSimState()::getSupplyCurrentMeasure);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motorClimber.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        motorClimber.setControl(request);
    }

    @Override
    public void setPosition(double newValue) {
        // Reset internal sim state
        currentPhysicsSim.setState(newValue, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateRotor();

        // Update internal raw position offset
        motorClimber.setPosition(newValue);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.connected = motorClimber.isConnected();
        inputs.positionMeters = motorClimber.getPosition().getValueAsDouble();
        inputs.velocityMps = motorClimber.getVelocity().getValueAsDouble();
        inputs.voltage = motorClimber.getMotorVoltage().getValueAsDouble();
        inputs.currentDraw = motorClimber.getSimState().getSupplyCurrentMeasure();
    }

    @Override
    public void close() {
        motorClimber.close();
    }

    public static Pose3d getSimulatedClimberPose(Pose2d robotPose, double climberHeightMeters) {
        Pose2d climberPose2d = robotPose.transformBy(Constants.Climber.ROBOT_TO_CLIMBER_OFFSET);
        return SimpleMath.withHeight(climberPose2d, climberHeightMeters + Constants.Climber.CLIMBER_BASE_HEIGHT_METERS);
    }

    public static boolean isWithinDistanceOfClimbing(
            double distanceMeters, Pose2d robotPose, double climberHeightMeters) {
        Pose3d climberPose = getSimulatedClimberPose(robotPose, climberHeightMeters);
        for (Translation3d towerEndPose : Constants.Climber.END_OF_TOWER_POSITIONS) {
            if (climberPose.getTranslation().getDistance(towerEndPose) < distanceMeters) {
                return true;
            }
        }
        return false;
    }

    private void updateRotor() {
        motorClimber
                .getSimState()
                .setRawRotorPosition(currentPhysicsSim.getPositionMeters() / Constants.Climber.METERS_PER_ROTATION);
        motorClimber
                .getSimState()
                .setRotorVelocity(
                        currentPhysicsSim.getVelocityMetersPerSecond() / Constants.Climber.METERS_PER_ROTATION);
    }

    @Override
    public void simulationPeriodic() {
        if (isWithinDistanceOfClimbing(
                        Constants.Climber.END_OF_TOWER_POSITION_TOLERANCE,
                        drivetrainSim.getSimulatedDriveTrainPose(),
                        currentPhysicsSim.getPositionMeters())
                && currentPhysicsSim == physicsSimNotSupportingRobot) {
            currentPhysicsSim = physicsSimSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimNotSupportingRobot.getPositionMeters(),
                    physicsSimNotSupportingRobot.getVelocityMetersPerSecond());
        } else if (!isWithinDistanceOfClimbing(
                        Constants.Climber.END_OF_TOWER_POSITION_TOLERANCE,
                        drivetrainSim.getSimulatedDriveTrainPose(),
                        currentPhysicsSim.getPositionMeters())
                && currentPhysicsSim == physicsSimSupportingRobot) {
            currentPhysicsSim = physicsSimNotSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimSupportingRobot.getPositionMeters(),
                    physicsSimSupportingRobot.getVelocityMetersPerSecond());
        }

        motorClimber.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = motorClimber.getSimState().getMotorVoltage();

        currentPhysicsSim.setInputVoltage(motorVoltage);
        currentPhysicsSim.update(periodicDt);

        updateRotor();
    }
}
