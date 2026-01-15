package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Climber extends KillableSubsystem implements PoweredSubsystem, EncoderResettableSubsystem {

    private static final double GOAL_POSITION_TOLERANCE = 0.01; // in rotations
    private static final double KV_ZERO_CLIP_THRESHOLD = 0.2;

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(4.0).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(3.5);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.0);

    private final ClimberIO io;
    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicVoltage armRequest;

    private ClimberState currentState = ClimberState.PARK;
    private double climbTarget = 0;

    private double positionCached = 0;
    private double velocityCached = 0;
    private double voltageCached = 0;

    private double lastClimbVoltage = 0.0;
    private double lastExpectedKVTime = 0;

    public Climber(ClimberIO io) {
        this.io = io;

        TalonFXConfiguration config = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Climber.KS;
        slot0Configs.kV = Constants.Climber.KV;
        slot0Configs.kA = Constants.Climber.KA;
        slot0Configs.kG = Constants.Climber.KG;
        slot0Configs.kP = Constants.Climber.KP;
        slot0Configs.kI = 0;
        slot0Configs.kD = Constants.Climber.KD;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        config.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climber.MAX_ARM_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Climber.MAX_ARM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Climber.MAX_ARM_JERK;

        io.applyTalonFXConfig(config.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        positionCached = io.getPosition();
        armRequest = new MotionMagicVoltage(Constants.Climber.START_ROTATIONS.in(Rotations));
        set(ClimberState.PARK);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Climber/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("Climber", Constants.Climber.START_ROTATIONS.in(Rotations));
        SmartDashboard.putBoolean("Ratchet", false);
    }

    public enum ClimberState {
        PARK,
        EXTEND,
        CLIMB
    }

    @Override
    public void periodicManaged() {
        positionCached = io.getPosition();
        velocityCached = io.getVelocity();
        voltageCached = io.getVoltage();

        if (currentState == ClimberState.CLIMB) {
            lastClimbVoltage = SimpleMath.slewRateLimitLinear(
                    lastClimbVoltage,
                    atGoal() ? 0 : Constants.Climber.CLIMB_VOLTAGE_MAX,
                    RobotContainer.ROBOT_PERIODIC,
                    Constants.Climber.CLIMB_VOLTAGE_SLEW_RATE);

            if (getEstimatedkV() >= Constants.Climber.CLIMB_EXPECTED_KV_MIN) {
                lastExpectedKVTime = Timer.getTimestamp();
            }

            if (Timer.getTimestamp() - lastExpectedKVTime > Constants.Climber.CLIMB_EXPECTED_KV_TIMEOUT.in(Seconds)
                    && !atGoal()) {
                // If we haven't seen a expected kV value in a while, set the voltage to 0 and park
                lastClimbVoltage = 0;
                set(ClimberState.PARK);
            } else {
                io.setVoltage(lastClimbVoltage);
            }
        }

        // Update mechanism
        RobotContainer.model.climber.update(getRotations());
    }

    @AutoLogLevel(level = Level.DEBUG_REAL)
    public boolean atGoal() {
        if (currentState == ClimberState.CLIMB) {
            return getRotations() >= climbTarget;
        } else if (currentState == ClimberState.EXTEND) {
            return Math.abs(getRotations() - Constants.Climber.EXTENDED_ROTATIONS.in(Rotations))
                    < GOAL_POSITION_TOLERANCE;
        } else {
            return Math.abs(getRotations() - Constants.Climber.PARK_ROTATIONS.in(Rotations)) < GOAL_POSITION_TOLERANCE;
        }
    }

    @SuppressWarnings("java:S1151") // switch statement is small
    public void set(ClimberState state) {
        currentState = state;
        switch (state) {
            case PARK -> {
                io.setRatchet(Constants.Climber.RATCHET_DISENGAGED);
                if (!(SysIdManager.getProvider() instanceof SysId)) {
                    io.setMotionMagic(armRequest.withPosition(Constants.Climber.PARK_ROTATIONS.in(Rotations)));
                }
            }
            case EXTEND -> {
                io.setRatchet(Constants.Climber.RATCHET_DISENGAGED);
                if (!(SysIdManager.getProvider() instanceof SysId)) {
                    io.setMotionMagic(armRequest.withPosition(Constants.Climber.EXTENDED_ROTATIONS.in(Rotations)));
                }
            }
            case CLIMB -> {
                climbTarget = Constants.Climber.CLIMBED_ROTATIONS.in(Rotations);
                io.setRatchet(Constants.Climber.RATCHET_ENGAGED);
                lastClimbVoltage = 0.0;
                lastExpectedKVTime = Timer.getTimestamp();
            }
        }
    }

    public void setBurst(Angle additionalAngle) {
        currentState = ClimberState.CLIMB;
        io.setRatchet(Constants.Climber.RATCHET_ENGAGED);
        lastClimbVoltage = 0.0;
        lastExpectedKVTime = Timer.getTimestamp();
        climbTarget += additionalAngle.in(Rotations);
    }

    @AutoLogLevel(level = Level.REAL)
    public ClimberState getCurrentState() {
        return currentState;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getRotations() {
        return positionCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getVelocity() {
        return velocityCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmSetTo() {
        return voltageCached;
    }

    @AutoLogLevel(level = Level.DEBUG_REAL)
    public double getEstimatedkV() {
        if (MathUtil.isNear(0, velocityCached, KV_ZERO_CLIP_THRESHOLD)) return 0;
        return voltageCached / velocityCached;
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public ClimberSim getSimIO() throws IllegalStateException {
        if (io instanceof ClimberSim simIO) {
            return simIO;
        } else {
            throw new IllegalStateException("ClimberIO is not a simulation");
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void kill() {
        io.setVoltage(0);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDrawAmps();
    }

    @Override
    public void resetEncoders() {
        io.setPosition(Constants.Climber.START_ROTATIONS.in(Rotations));
        positionCached = Constants.Climber.START_ROTATIONS.in(Rotations);
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.climber.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.climber.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
