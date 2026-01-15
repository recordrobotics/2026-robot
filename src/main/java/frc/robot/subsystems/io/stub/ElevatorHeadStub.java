package frc.robot.subsystems.io.stub;

import frc.robot.subsystems.io.ElevatorHeadIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class ElevatorHeadStub implements ElevatorHeadIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ElevatorHeadStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void setVoltage(double outputVolts) {}

    @Override
    public void setPosition(double newValue) {}

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public void setPercent(double newValue) {}

    @Override
    public double getPercent() {
        return 0;
    }

    @Override
    public boolean isCoralDetectorTriggered() {
        return false;
    }

    @Override
    public double getCurrentDrawAmps() {
        return 0;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
