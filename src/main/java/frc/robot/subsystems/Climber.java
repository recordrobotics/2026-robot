package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ClimberHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public final class Climber extends KillableSubsystem implements PoweredSubsystem, EncoderResettableSubsystem {

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE_FREE =
            Volts.of(4.5).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE_FREE = Volts.of(3.0);
    private static final Time SYSID_TIMEOUT_FREE = Seconds.of(1.2);

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE_LOADED =
            Volts.of(4.5).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE_LOADED = Volts.of(3.0);
    private static final Time SYSID_TIMEOUT_LOADED = Seconds.of(1.2);

    private final ClimberIO io;

    private final MotionMagicExpoVoltage climberRequest;

    private double setpoint;

    private final SysIdRoutine sysIdRoutineFree;
    private final SysIdRoutine sysIdRoutineLoaded;

    private boolean supportingRobot = false;

    public Climber(ClimberIO io) {
        this.io = io;

        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.Feedback.SensorToMechanismRatio = 1.0 / Constants.Climber.METERS_PER_ROTATION;

        // slot 0 gains for when climber is not supporting weight of robot
        Slot0Configs slot0ConfigsClimber = climberConfig.Slot0;
        slot0ConfigsClimber.kS = Constants.Climber.KS;
        slot0ConfigsClimber.kV = Constants.Climber.KV_0;
        slot0ConfigsClimber.kA = Constants.Climber.KA_0;
        slot0ConfigsClimber.kG = Constants.Climber.KG_0;
        slot0ConfigsClimber.kP = Constants.Climber.KP_0;
        slot0ConfigsClimber.kD = Constants.Climber.KD_0;
        slot0ConfigsClimber.GravityType = GravityTypeValue.Elevator_Static;

        // slot 1 gains for when climber is supporting weight of robot
        Slot1Configs slot1ConfigsClimber = climberConfig.Slot1;
        slot1ConfigsClimber.kS = Constants.Climber.KS;
        slot1ConfigsClimber.kV = Constants.Climber.KV_1;
        slot1ConfigsClimber.kA = Constants.Climber.KA_1;
        slot1ConfigsClimber.kG = Constants.Climber.KG_1;
        slot1ConfigsClimber.kP = Constants.Climber.KP_1;
        slot1ConfigsClimber.kD = Constants.Climber.KD_1;
        slot1ConfigsClimber.GravityType = GravityTypeValue.Elevator_Static;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsClimber = climberConfig.MotionMagic;
        motionMagicConfigsClimber.MotionMagicExpo_kV = Constants.Climber.MMEXPO_KV_0;
        motionMagicConfigsClimber.MotionMagicExpo_kA = Constants.Climber.MMEXPO_KA_0;

        io.applyTalonFXConfig(climberConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                        .withSupplyCurrentLowerLimit(Constants.Climber.SUPPLY_CURRENT_LOWER_LIMIT)
                        .withSupplyCurrentLowerTime(1)
                        .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        climberRequest = new MotionMagicExpoVoltage(0).withSlot(0);

        setState(ClimberHeight.DOWN);

        sysIdRoutineFree = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE_FREE,
                        SYSID_STEP_VOLTAGE_FREE,
                        SYSID_TIMEOUT_FREE,
                        state -> Logger.recordOutput("Climber/Free/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

        sysIdRoutineLoaded = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE_LOADED,
                        SYSID_STEP_VOLTAGE_LOADED,
                        SYSID_TIMEOUT_LOADED,
                        state -> Logger.recordOutput("Climber/Loaded/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));
    }

    /** Height of the climber in meters */
    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentHeight() {
        return io.getPosition();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentVelocity() {
        return io.getVelocity();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentVoltage() {
        return io.getVoltage();
    }

    @Override
    public void periodicManaged() {
        // Update mechanism
        RobotContainer.model.climberModel.update(getCurrentHeight());

        if (io.getCurrentDraw() > Constants.Climber.STATOR_CURRENT_AMPS_THRESHOLD
                && !supportingRobot) { // TODO maybe debounce over time?
            supportingRobot = true;
        } else if (io.getCurrentDraw() < Constants.Climber.STATOR_CURRENT_AMPS_THRESHOLD
                && supportingRobot) { // TODO maybe debounce over amps? (different threshold for up and down)
            supportingRobot = false;
        }
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public void setState(ClimberHeight height) {
        setpoint = height.getHeight();

        if (!isForceDisabled()
                && !(SysIdManager.getProvider() instanceof SysIdFree)
                && !(SysIdManager.getProvider() instanceof SysIdLoaded)) {
            io.setMotionMagic(climberRequest.withPosition(setpoint).withSlot(supportingRobot ? 1 : 0));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setVoltage(0.0);
        } else {
            io.setMotionMagic(climberRequest.withPosition(setpoint).withSlot(supportingRobot ? 1 : 0));
        }
    }

    @AutoLogLevel(level = Level.REAL)
    public ClimberHeight getNearestHeight() {
        double currentHeight = getCurrentHeight();

        ClimberHeight[] heights = ClimberHeight.values();
        Arrays.sort(heights, (a, b) -> Double.compare(a.getDifference(currentHeight), b.getDifference(currentHeight)));
        return heights[0];
    }

    public boolean atGoal() {
        return Math.abs(setpoint - getCurrentHeight()) < Constants.Climber.AT_GOAL_POSITION_TOLERANCE
                && Math.abs(getCurrentVelocity()) < Constants.Climber.AT_GOAL_VELOCITY_TOLERANCE;
    }

    public Command sysIdQuasistaticFree(SysIdRoutine.Direction direction) {
        return sysIdRoutineFree.quasistatic(direction);
    }

    public Command sysIdDynamicFree(SysIdRoutine.Direction direction) {
        return sysIdRoutineFree.dynamic(direction);
    }

    public Command sysIdQuasistaticLoaded(SysIdRoutine.Direction direction) {
        return sysIdRoutineLoaded.quasistatic(direction);
    }

    public Command sysIdDynamicLoaded(SysIdRoutine.Direction direction) {
        return sysIdRoutineLoaded.dynamic(direction);
    }

    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDraw();
    }

    @Override
    public void resetEncoders() {
        io.setPosition(0);
    }

    public static class SysIdFree implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.climber.sysIdQuasistaticFree(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.climber.sysIdDynamicFree(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }

    public static class SysIdLoaded implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.climber.sysIdQuasistaticLoaded(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.climber.sysIdDynamicLoaded(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
