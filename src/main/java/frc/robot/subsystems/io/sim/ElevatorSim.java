package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorSim implements ElevatorIO {

    private final double periodicDt;

    private final TalonFX motorLead;
    private final TalonFX motorFollower;

    private final TalonFXSimState motorLeadSim;
    private final TalonFXSimState motorFollowerSim;

    private final edu.wpi.first.wpilibj.simulation.ElevatorSim physicsSim =
            new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                    DCMotor.getKrakenX60(2),
                    Constants.Elevator.GEAR_RATIO,
                    1.0,
                    Constants.Elevator.DRUM_RADIUS,
                    0,
                    Constants.Elevator.MAX_HEIGHT,
                    true,
                    0,
                    0.0003,
                    0.0003);

    private final DigitalInput bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
    private final DigitalInput topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);
    private final SimDevice bottomEndStopSim = SimDevice.create("DigitalInput", RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
    private final SimDevice topEndStopSim = SimDevice.create("DigitalInput", RobotMap.Elevator.TOP_ENDSTOP_ID);
    private final SimBoolean bottomEndStopSimValue;
    private final SimBoolean topEndStopSimValue;

    public ElevatorSim(double periodicDt) {
        this.periodicDt = periodicDt;

        motorLead = new TalonFX(RobotMap.Elevator.MOTOR_LEAD_ID);
        motorFollower = new TalonFX(RobotMap.Elevator.MOTOR_FOLLOWER_ID);

        motorLeadSim = motorLead.getSimState();
        motorFollowerSim = motorFollower.getSimState();

        motorLeadSim.Orientation = ChassisReference.Clockwise_Positive;
        motorFollowerSim.Orientation = ChassisReference.Clockwise_Positive;

        if (bottomEndStopSim != null)
            bottomEndStopSimValue = bottomEndStopSim.createBoolean("Value", Direction.kOutput, false);
        else bottomEndStopSimValue = null;

        if (topEndStopSim != null) topEndStopSimValue = topEndStopSim.createBoolean("Value", Direction.kOutput, false);
        else topEndStopSimValue = null;

        if (bottomEndStopSim != null) bottomEndStop.setSimDevice(bottomEndStopSim);
        else bottomEndStop.close();

        if (topEndStopSim != null) topEndStop.setSimDevice(topEndStopSim);
        else topEndStop.close();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motorLead.getConfigurator().apply(configuration);
        motorFollower.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createFollower() {
        return new Follower(RobotMap.Elevator.MOTOR_LEAD_ID, false);
    }

    @Override
    public void setLeadMotorVoltage(double outputVolts) {
        motorLead.setVoltage(outputVolts);
    }

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {
        motorLead.setControl(request);
    }

    @Override
    public void setFollowerMotionMagic(Follower request) {
        motorFollower.setControl(request);
    }

    @Override
    public double getLeadMotorVoltage() {
        return motorLead.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorVoltage() {
        return motorFollower.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setLeadMotorPosition(double newValue) {
        // Reset internal sim state
        physicsSim.setState(newValue, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        motorLeadSim.setRawRotorPosition(physicsSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
        motorLeadSim.setRotorVelocity(physicsSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
        motorFollowerSim.setRawRotorPosition(physicsSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
        motorFollowerSim.setRotorVelocity(
                physicsSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);

        // Update internal raw position offset
        motorLead.setPosition(newValue);
        motorFollower.setPosition(newValue);
    }

    @Override
    public void setFollowerMotorPosition(double newValue) {
        setLeadMotorPosition(newValue);
    }

    @Override
    public double getLeadMotorPosition() {
        return motorLead.getPosition().getValueAsDouble();
    }

    @Override
    public double getLeadMotorVelocity() {
        return motorLead.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorPosition() {
        return motorFollower.getPosition().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorVelocity() {
        return motorFollower.getVelocity().getValueAsDouble();
    }

    @Override
    public void setLeadMotorPercent(double newValue) {
        motorLead.set(newValue);
    }

    @Override
    public double getLeadMotorPercent() {
        return motorLead.get();
    }

    @Override
    public boolean isTopEndStopPressed() {
        if (topEndStopSimValue != null) return topEndStopSimValue.get();
        else return false;
    }

    @Override
    public boolean isBottomEndStopPressed() {
        if (bottomEndStopSimValue != null) return bottomEndStopSimValue.get();
        else return false;
    }

    public void setTopEndStopSim(boolean newValue) {
        if (topEndStopSimValue != null) topEndStopSimValue.set(newValue);
    }

    public void setBottomEndStopSim(boolean newValue) {
        if (bottomEndStopSimValue != null) bottomEndStopSimValue.set(newValue);
    }

    @Override
    public double getLeadMotorCurrentDraw() {
        return motorLead.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorCurrentDraw() {
        return motorFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        motorLead.close();
        motorFollower.close();

        if (bottomEndStopSim != null) {
            bottomEndStopSim.close();
            bottomEndStop.close();
        }
        if (topEndStopSim != null) {
            topEndStopSim.close();
            topEndStop.close();
        }
    }

    @Override
    public void simulationPeriodic() {
        motorLeadSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorLeftVoltage = motorLeadSim.getMotorVoltage();
        double motorRightVoltage = motorFollowerSim.getMotorVoltage();

        physicsSim.setInputVoltage((motorLeftVoltage + motorRightVoltage) / 2.0);
        physicsSim.update(periodicDt);

        motorLeadSim.setRawRotorPosition(physicsSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
        motorLeadSim.setRotorVelocity(physicsSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
        motorFollowerSim.setRawRotorPosition(physicsSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
        motorFollowerSim.setRotorVelocity(
                physicsSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
    }
}
