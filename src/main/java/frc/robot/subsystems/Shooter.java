package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends KillableSubsystem implements PoweredSubsystem {

    private static final double HOOD_POSITION_TOLERANCE = Units.degreesToRotations(2);
    private static final double HOOD_VELOCITY_TOLERANCE = Units.degreesToRotations(20);

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(2.0).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(1.5);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.3);

    private static final double FLYWHEEL_VELOCITY_TOLERANCE_MPS = 10; // TODO

    private static final Distance FLYWHEEL_WHEEL_DIAMETER = Inches.of(4);

    private final ShooterIO io;
    private final SysIdRoutine sysIdRoutineFlywheel;
    private final SysIdRoutine sysIdRoutineHood;
    private final MotionMagicExpoVoltage hoodRequest;
    private final MotionMagicVelocityVoltage flywheelRequest;
    private final Follower flywheelFollowerRequest;

    private double hoodTargetPositionRotations;
    private double flywheelTargetVelocityMps;

    public Shooter(ShooterIO io) {
        this.io = io;
        hoodRequest =
                new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
        flywheelRequest = new MotionMagicVelocityVoltage(0.0);
        flywheelFollowerRequest = io.createFlywheelFollower();

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        Slot0Configs hoodSlot0Configs = hoodConfig.Slot0;
        hoodSlot0Configs.kS = Constants.Shooter.HOOD_KS;
        hoodSlot0Configs.kV = Constants.Shooter.HOOD_KV;
        hoodSlot0Configs.kA = Constants.Shooter.HOOD_KA;
        hoodSlot0Configs.kP = Constants.Shooter.HOOD_KP;
        hoodSlot0Configs.kD = Constants.Shooter.HOOD_KD;

        hoodConfig.MotionMagic.MotionMagicExpo_kV = Constants.Shooter.HOOD_MMEXPO_KV;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = Constants.Shooter.HOOD_MMEXPO_KA;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.HOOD_SUPPLY_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Shooter.HOOD_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Shooter.HOOD_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.HOOD_STATOR_CURRENT_LIMIT.in(Amps);
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(Constants.Shooter.HOOD_MAX_POSITION_RADIANS);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(Constants.Shooter.HOOD_MIN_POSITION_RADIANS);
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.HOOD_GEAR_RATIO;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        io.applyHoodTalonFXConfig(hoodConfig);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        Slot0Configs flywheelSlot0Configs = flywheelConfig.Slot0;
        flywheelSlot0Configs.kS = Constants.Shooter.FLYWHEEL_KS;
        flywheelSlot0Configs.kV = Constants.Shooter.FLYWHEEL_KV;
        flywheelSlot0Configs.kA = Constants.Shooter.FLYWHEEL_KA;
        flywheelSlot0Configs.kP = Constants.Shooter.FLYWHEEL_KP;

        flywheelConfig.MotionMagic.MotionMagicJerk = Constants.Shooter.FLYWHEEL_MAX_JERK;
        flywheelConfig.MotionMagic.MotionMagicAcceleration = Constants.Shooter.FLYWHEEL_MAX_ACCELERATION;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.FLYWHEEL_SUPPLY_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit =
                Constants.Shooter.FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Shooter.FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.FLYWHEEL_STATOR_CURRENT_LIMIT.in(Amps);
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        flywheelConfig.Feedback.SensorToMechanismRatio =
                Constants.Shooter.FLYWHEEL_GEAR_RATIO * Math.PI * FLYWHEEL_WHEEL_DIAMETER.in(Meters);
        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        io.applyFlywheelLeaderTalonFXConfig(flywheelConfig);
        io.applyFlywheelFollowerTalonFXConfig(flywheelConfig.withMotorOutput(
                flywheelConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)));
        io.setFlywheelFollowerMotionMagic(flywheelFollowerRequest);

        setTargetState(new ShooterState(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS, 0.0));

        sysIdRoutineFlywheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("Shooter/Flywheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setFlywheelVoltage(v.in(Volts)), null, this));

        sysIdRoutineHood = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Shooter/Hood/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setHoodVoltage(v.in(Volts)), null, this));
    }

    public void setTargetState(ShooterState targetState) {
        hoodTargetPositionRotations = Units.radiansToRotations(targetState.hoodAngle);
        flywheelTargetVelocityMps = targetState.flywheelVelocityMps;

        if (!isForceDisabled()) {
            if (!(SysIdManager.getProvider() instanceof SysIdHood))
                io.setHoodMotionMagic(hoodRequest.withPosition(hoodTargetPositionRotations));
            if (!(SysIdManager.getProvider() instanceof SysIdFlywheel))
                io.setFlywheelMotionMagic(flywheelRequest.withVelocity(flywheelTargetVelocityMps));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setHoodVoltage(0.0);
            io.setFlywheelVoltage(0.0);
        } else {
            io.setHoodMotionMagic(hoodRequest.withPosition(hoodTargetPositionRotations));
            io.setFlywheelMotionMagic(flywheelRequest.withVelocity(flywheelTargetVelocityMps));
        }
    }

    @Override
    public void periodicManaged() {
        RobotContainer.model.shooterModel.updateHood(Units.rotationsToRadians(getHoodPositionRotations()));
    }

    public boolean isAtTargetState() {
        return SimpleMath.isWithinTolerance(
                        getHoodPositionRotations(), hoodTargetPositionRotations, HOOD_POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(getHoodVelocityRotationsPerSecond(), 0, HOOD_VELOCITY_TOLERANCE)
                && SimpleMath.isWithinTolerance(
                        getFlywheelVelocityMps(), flywheelTargetVelocityMps, FLYWHEEL_VELOCITY_TOLERANCE_MPS);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getTargetHoodAngle() {
        return Units.rotationsToRadians(hoodTargetPositionRotations);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getTargetFlywheelVelocityMps() {
        return flywheelTargetVelocityMps;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getHoodAngle() {
        return Units.rotationsToRadians(io.getHoodPositionRotations());
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getHoodPositionRotations() {
        return io.getHoodPositionRotations();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getHoodVelocityRotationsPerSecond() {
        return io.getHoodVelocityRotationsPerSecond();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getHoodVoltage() {
        return io.getHoodVoltage();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getFlywheelPositionMeters() {
        return (io.getFlywheelLeaderPositionMeters() + io.getFlywheelFollowerPositionMeters()) / 2.0;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public double getFlywheelVelocityMps() {
        return (io.getFlywheelLeaderVelocityMps() + io.getFlywheelFollowerVelocityMps()) / 2.0;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getFlywheelVoltage() {
        return (Math.abs(io.getFlywheelLeaderVoltage()) + Math.abs(io.getFlywheelFollowerVoltage())) / 2.0;
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getFlywheelLeaderCurrentDrawAmps()
                + io.getFlywheelFollowerCurrentDrawAmps()
                + io.getHoodCurrentDrawAmps();
    }

    public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.quasistatic(direction);
    }

    public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineFlywheel.dynamic(direction);
    }

    public Command sysIdQuasistaticHood(SysIdRoutine.Direction direction) {
        return sysIdRoutineHood.quasistatic(direction);
    }

    public Command sysIdDynamicHood(SysIdRoutine.Direction direction) {
        return sysIdRoutineHood.dynamic(direction);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public void resetEncoders() {
        io.setHoodPositionRotations(Units.radiansToRotations(Constants.Shooter.HOOD_STARTING_POSITION_RADIANS));
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public record ShooterState(double hoodAngle, double flywheelVelocityMps) {}

    public static class SysIdHood implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.shooter.sysIdQuasistaticHood(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.shooter.sysIdDynamicHood(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }

    public static class SysIdFlywheel implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.shooter.sysIdQuasistaticFlywheel(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.shooter.sysIdDynamicFlywheel(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
