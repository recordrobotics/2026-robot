package frc.robot.subsystems.io.real;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorHeadIO;

public class ElevatorHeadReal implements ElevatorHeadIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final SparkMax motor;
    private final DigitalInput coralDetector = new DigitalInput(RobotMap.ElevatorHead.PHOTOSENSOR_ID);

    public ElevatorHeadReal(double periodicDt) {
        this.periodicDt = periodicDt;
        motor = new SparkMax(RobotMap.ElevatorHead.MOTOR_ID, MotorType.kBrushless);
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
        return coralDetector.get();
    }

    @Override
    public double getCurrentDrawAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void close() throws Exception {
        motor.close();
        coralDetector.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
