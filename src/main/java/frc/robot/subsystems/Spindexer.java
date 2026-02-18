package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.io.SpindexerIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;

public final class Spindexer extends KillableSubsystem implements PoweredSubsystem {

    private static final double VELOCITY_TOLERANCE_RPS = 15.0; // TODO

    private final SpindexerIO io;
    private final MotionMagicVelocityVoltage request;

    private double targetVelocityRps;
    private SpindexerState targetState = SpindexerState.OFF;

    public enum SpindexerState {
        OFF,
        ON
    }

    public Spindexer(SpindexerIO io) {
        this.io = io;
        request = new MotionMagicVelocityVoltage(0.0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Spindexer.KS;
        slot0Configs.kV = Constants.Spindexer.KV;
        slot0Configs.kA = Constants.Spindexer.KA;
        slot0Configs.kP = Constants.Spindexer.KP;

        config.MotionMagic.MotionMagicJerk = Constants.Spindexer.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration = Constants.Spindexer.MAX_ACCELERATION;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Spindexer.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Spindexer.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Spindexer.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Spindexer.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        io.applyTalonFXConfig(config);

        setState(SpindexerState.OFF);
    }

    public void setState(SpindexerState newState) {
        targetState = newState;

        if (targetState == SpindexerState.OFF) {
            targetVelocityRps = 0.0;
        } else if (targetState == SpindexerState.ON) {
            targetVelocityRps = Constants.Spindexer.INTAKE_VELOCITY_RPS;
        }

        io.setMotionMagic(request.withVelocity(targetVelocityRps));
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public SpindexerState getTargetState() {
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
