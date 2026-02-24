package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.utils.SimpleMath;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class ClimberSim implements ClimberIO {

    private final double periodicDt;

    private final TalonFX motorClimber;

    private final TalonFXSimState motorClimberSim;

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

        motorClimberSim = motorClimber.getSimState();

        motorClimberSim.Orientation = ChassisReference.Clockwise_Positive; // correct
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motorClimber.getConfigurator().apply(configuration);
    }

    @Override
    public void setVoltage(double outputVolts) {
        motorClimber.setVoltage(outputVolts);
    }

    @Override
    public void setMotionMagic(MotionMagicExpoVoltage request) {
        motorClimber.setControl(request);
    }

    @Override
    public double getVoltage() {
        return motorClimber.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setPosition(double newValue) {
        // Reset internal sim state
        currentPhysicsSim.setState(newValue, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        motorClimberSim.setRawRotorPosition(
                currentPhysicsSim.getPositionMeters() / Constants.Climber.METERS_PER_ROTATION);
        motorClimberSim.setRotorVelocity(
                currentPhysicsSim.getVelocityMetersPerSecond() / Constants.Climber.METERS_PER_ROTATION);

        // Update internal raw position offset
        motorClimber.setPosition(newValue);
    }

    @Override
    public double getPosition() {
        return motorClimber.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return motorClimber.getVelocity().getValueAsDouble();
    }

    @Override
    public double getCurrentDraw() {
        return motorClimber.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        motorClimber.close();
    }

    private Pose3d getSimulatedClimberPose() {
        edu.wpi.first.math.geometry.Pose2d climberPose2d =
                drivetrainSim.getSimulatedDriveTrainPose().transformBy(Constants.Climber.ROBOT_TO_CLIMBER_OFFSET);
        return SimpleMath.withHeight(
                climberPose2d, currentPhysicsSim.getPositionMeters() + Constants.Climber.CLIMBER_BASE_HEIGHT_METERS);
    }

    private boolean isClimbing() {
        Pose3d climberPose = getSimulatedClimberPose();
        for (Translation3d towerEndPose : Constants.Climber.END_OF_TOWER_POSITIONS) {
            if (climberPose.getTranslation().getDistance(towerEndPose)
                    < Constants.Climber.END_OF_TOWER_POSITION_TOLERANCE) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void simulationPeriodic() {
        if (isClimbing() && currentPhysicsSim == physicsSimNotSupportingRobot) {
            currentPhysicsSim = physicsSimSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimNotSupportingRobot.getPositionMeters(),
                    physicsSimNotSupportingRobot.getVelocityMetersPerSecond());
        } else if (!isClimbing() && currentPhysicsSim == physicsSimSupportingRobot) {
            currentPhysicsSim = physicsSimNotSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimSupportingRobot.getPositionMeters(),
                    physicsSimSupportingRobot.getVelocityMetersPerSecond());
        }

        motorClimberSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = motorClimberSim.getMotorVoltage();

        currentPhysicsSim.setInputVoltage(motorVoltage);
        currentPhysicsSim.update(periodicDt);

        motorClimberSim.setRawRotorPosition(
                currentPhysicsSim.getPositionMeters() / Constants.Climber.METERS_PER_ROTATION);
        motorClimberSim.setRotorVelocity(
                currentPhysicsSim.getVelocityMetersPerSecond() / Constants.Climber.METERS_PER_ROTATION);
    }
}
