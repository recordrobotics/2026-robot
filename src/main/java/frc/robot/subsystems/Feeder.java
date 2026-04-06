package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.FeederIO;
import frc.robot.subsystems.io.sim.FeederSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.CircularEventCounter;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Feeder extends KillableSubsystem implements PoweredSubsystem {

    private static final double VELOCITY_TOLERANCE_RPS = 15.0; // TODO

    private static final double BEAM_BREAK_FAULT_TIME_SECONDS = 0.5;
    private static final int BEAM_BREAK_DISCONNECTED_BROKEN_COUNT = 3;
    private static final int BEAM_BREAK_BROKEN_MAX_COUNT = 10; // no way we are getting above 10 bps

    private final FeederIO io;
    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicVelocityVoltage request;

    private double targetVelocityRps;
    private FeederState targetState = FeederState.OFF;

    private final Alert bottomBeambreakFaultAlert = new Alert("Feeder bottom beambreak fault", Alert.AlertType.kError);
    private final Alert topBeambreakFaultAlert = new Alert("Feeder top beambreak fault", Alert.AlertType.kError);
    private final Alert bottomBeambreakDisconnectedAlert =
            new Alert("Feeder bottom beambreak disconnected", Alert.AlertType.kError);
    private final Alert topBeambreakDisconnectedAlert =
            new Alert("Feeder top beambreak disconnected", Alert.AlertType.kError);

    private double lastBottomBeamNotbrokenTime = 0;
    private double lastTopBeamNotbrokenTime = 0;
    private boolean bottomBeamFaulted = false;
    private boolean topBeamFaulted = false;

    private CircularEventCounter bottomBeamBrokenCounter = new CircularEventCounter(BEAM_BREAK_BROKEN_MAX_COUNT, 1.0);
    private CircularEventCounter topBeamBrokenCounter = new CircularEventCounter(BEAM_BREAK_BROKEN_MAX_COUNT, 1.0);
    private boolean lastBottomBeamBroken = false;
    private boolean lastTopBeamBroken = false;

    public enum FeederState {
        OFF,
        ON,
        UNSTUCK
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

    public FeederSim getSimIO() {
        if (io instanceof FeederSim sim) {
            return sim;
        }

        return null;
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
    public void periodicManaged() {
        if (!io.isBottomBeamBroken()) {
            lastBottomBeamNotbrokenTime = Timer.getTimestamp();
        }

        if (!io.isTopBeamBroken()) {
            lastTopBeamNotbrokenTime = Timer.getTimestamp();
        }

        if (Timer.getTimestamp() - lastBottomBeamNotbrokenTime > BEAM_BREAK_FAULT_TIME_SECONDS) {
            bottomBeamFaulted = true;
            bottomBeambreakFaultAlert.set(true);
        } else {
            bottomBeamFaulted = false;
            bottomBeambreakFaultAlert.set(false);
        }

        if (Timer.getTimestamp() - lastTopBeamNotbrokenTime > BEAM_BREAK_FAULT_TIME_SECONDS) {
            topBeamFaulted = true;
            topBeambreakFaultAlert.set(true);
        } else {
            topBeamFaulted = false;
            topBeambreakFaultAlert.set(false);
        }

        if (isBottomBeamBroken() && !lastBottomBeamBroken) {
            bottomBeamBrokenCounter.recordEvent();
        }
        lastBottomBeamBroken = isBottomBeamBroken();

        if (isTopBeamBroken() && !lastTopBeamBroken) {
            topBeamBrokenCounter.recordEvent();
        }
        lastTopBeamBroken = isTopBeamBroken();

        int topCount = topBeamBrokenCounter.getEventCount();
        int bottomCount = bottomBeamBrokenCounter.getEventCount();

        Logger.recordOutput("Feeder/TopBPS", topCount);
        Logger.recordOutput("Feeder/BottomBPS", bottomCount);

        if (bottomCount >= BEAM_BREAK_DISCONNECTED_BROKEN_COUNT && topCount == 0) {
            topBeambreakDisconnectedAlert.set(true);
        } else if (isTopBeamBroken()) {
            topBeambreakDisconnectedAlert.set(false);
        }

        if (topCount >= BEAM_BREAK_DISCONNECTED_BROKEN_COUNT && bottomCount == 0) {
            bottomBeambreakDisconnectedAlert.set(true);
        } else if (isBottomBeamBroken()) {
            bottomBeambreakDisconnectedAlert.set(false);
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
        return io.isBottomBeamBroken() && !bottomBeamFaulted;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public boolean isTopBeamBroken() {
        return io.isTopBeamBroken() && !topBeamFaulted;
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
    public Current getCurrentDraw() {
        return io.getCurrentDraw();
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
