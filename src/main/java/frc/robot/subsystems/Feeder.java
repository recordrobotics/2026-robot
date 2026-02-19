package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.io.FeederIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;

public final class Feeder extends KillableSubsystem implements PoweredSubsystem {

    private static final double VELOCITY_TOLERANCE_RPS = 15.0; // TODO

    private final FeederIO io;
    private final MotionMagicVelocityVoltage request;

    private double targetVelocityRps;
    private FeederState targetState = FeederState.OFF;

    public enum FeederState {
        OFF,
        ON
    }

    public Feeder(FeederIO io) {
        this.io = io;
        request = new MotionMagicVelocityVoltage(0.0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Feeder.KS;
        slot0Configs.kV = Constants.Feeder.KV;
        slot0Configs.kA = Constants.Feeder.KA;
        slot0Configs.kP = Constants.Feeder.KP;

        config.MotionMagic.MotionMagicJerk = Constants.Feeder.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration = Constants.Feeder.MAX_ACCELERATION;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Feeder.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Feeder.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Feeder.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Feeder.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Feeder.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        io.applyTalonFXConfig(config);

        setState(FeederState.OFF);
    }

    public void setState(FeederState newState) {
        targetState = newState;

        if (targetState == FeederState.OFF) {
            targetVelocityRps = 0.0;
        } else if (targetState == FeederState.ON) {
            targetVelocityRps = Constants.Feeder.INTAKE_VELOCITY_RPS;
        }

        io.setMotionMagic(request.withVelocity(targetVelocityRps));
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public FeederState getTargetState() {
        return targetState;
    }

    public boolean atGoal() {
        return SimpleMath.isWithinTolerance(getVelocityRps(), targetVelocityRps, VELOCITY_TOLERANCE_RPS);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getPositionRotations() {
        return io.getPositionRotations();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVelocityRps() {
        return io.getVelocityRotationsPerSecond();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVoltage() {
        return io.getVoltage();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    @Override
    public void kill() {
        io.setVoltage(0);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }
}
