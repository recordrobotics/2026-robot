package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.real.ClimberReal;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class ClimberSim extends ClimberReal {

    private final double periodicDt;

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

    private ElevatorSim currentPhysicsSim = physicsSimNotSupportingRobot;

    public ClimberSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;

        motor.getSimState().Orientation = ChassisReference.Clockwise_Positive; // correct

        RobotContainer.pdp.registerSimDevice(12, motor.getSimState()::getSupplyCurrentMeasure);
    }

    @Override
    public void setPosition(double newValue) {
        // Reset internal sim state
        currentPhysicsSim.setState(newValue, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateRotor();

        super.setPosition(newValue);
    }

    private void updateRotor() {
        motor.getSimState()
                .setRawRotorPosition(currentPhysicsSim.getPositionMeters() / Constants.Climber.METERS_PER_ROTATION);
        motor.getSimState()
                .setRotorVelocity(
                        currentPhysicsSim.getVelocityMetersPerSecond() / Constants.Climber.METERS_PER_ROTATION);
    }

    @Override
    public void simulationPeriodic() {
        motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = motor.getSimState().getMotorVoltage();

        currentPhysicsSim.setInputVoltage(motorVoltage);
        currentPhysicsSim.update(periodicDt);

        updateRotor();
    }
}
