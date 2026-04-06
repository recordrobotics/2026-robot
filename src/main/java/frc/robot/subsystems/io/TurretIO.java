package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Current;

public interface TurretIO {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setControl(ControlRequest request);

    boolean hasHitForwardSoftLimit();

    boolean hasHitReverseSoftLimit();

    double getPositionRotations();

    double getVelocityRotationsPerSecond();

    double getVoltage();

    Current getCurrentDraw();

    void setPositionRotations(double newValue);

    LimitSwitchStates getLimitSwitchStates();

    void close();

    void simulationPeriodic();

    record LimitSwitchStates(boolean frontLeft, boolean backLeft, boolean backRight) {

        public static final LimitSwitchStates NO_HITS = new LimitSwitchStates(false, false, false);

        public boolean isAnyHit() {
            return frontLeft || backLeft || backRight;
        }

        public boolean hasFault() {
            // If more than one switch is hit at the same time, it's likely a fault (e.g. wiring issue)
            return (frontLeft ? 1 : 0) + (backLeft ? 1 : 0) + (backRight ? 1 : 0) > 1;
        }
    }
}
