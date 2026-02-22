package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.FeederIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Feeder extends KillableSubsystem implements PoweredSubsystem {

    private static final double VELOCITY_TOLERANCE_RPS = 15.0; // TODO

    private final FeederIO io;
    private final SysIdRoutine sysIdRoutine;
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

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("Feeder/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));
    }

    public void setState(FeederState newState) {
        targetState = newState;

        if (targetState == FeederState.OFF) {
            targetVelocityRps = 0.0;
        } else if (targetState == FeederState.ON) {
            targetVelocityRps = Constants.Feeder.INTAKE_VELOCITY_RPS;
        }

        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysId)) {
            io.setMotionMagic(request.withVelocity(targetVelocityRps));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setVoltage(0.0);
        } else {
            io.setMotionMagic(request.withVelocity(targetVelocityRps));
        }
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public FeederState getTargetState() {
        return targetState;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public boolean isBottomBeamBroken() {
        return io.isBottomBeamBroken();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public boolean isTopBeamBroken() {
        return io.isTopBeamBroken();
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.feeder.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.feeder.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }
}
