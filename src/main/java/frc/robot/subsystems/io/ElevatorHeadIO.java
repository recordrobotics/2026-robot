package frc.robot.subsystems.io;

public interface ElevatorHeadIO extends AutoCloseable {

    void setVoltage(double outputVolts);

    void setPosition(double newValue);

    double getPosition();

    double getVelocity();

    double getVoltage();

    void setPercent(double newValue);

    double getPercent();

    boolean isCoralDetectorTriggered();

    double getCurrentDrawAmps();

    void simulationPeriodic();
}
