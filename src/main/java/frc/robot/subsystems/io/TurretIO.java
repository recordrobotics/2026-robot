package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface TurretIO {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void setMotionMagic(MotionMagicExpoVoltage request);

    void setVoltage(double newValue);

    double getPositionRotations();

    double getVelocityRotationsPerSecond();

    double getVoltage();

    double getCurrentDrawAmps();

    void setPositionRotations(double newValue);

    LimitSwitchStates getLimitSwitchStates();

    void close();

    void simulationPeriodic();

    record LimitSwitchStates(boolean frontLeft, boolean backLeft, boolean backRight) {

        public static final LimitSwitchStates NO_HITS = new LimitSwitchStates(false, false, false);

        public boolean isAnyHit() {
            return frontLeft || backLeft || backRight;
        }
    }
}
