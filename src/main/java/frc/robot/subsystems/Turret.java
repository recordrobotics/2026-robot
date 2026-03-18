package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.TurretIO;
import frc.robot.subsystems.io.TurretIO.LimitSwitchStates;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Turret extends KillableSubsystem implements PoweredSubsystem {

    public static final double MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS = Units.degreesToRotations(90);

    private static final double POSITION_TOLERANCE = Units.degreesToRotations(12);
    private static final double VELOCITY_TOLERANCE = Units.degreesToRotations(500);

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(2.7).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(1.0);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.0);

    private static final double TWO_PI = 2.0 * Math.PI;

    private final TurretIO io;
    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicExpoVoltage turretRequest;
    private double targetPositionRotations;
    private double targetVelocityRotationsPerSecond;
    private double targetAccelerationRotationsPerSecondSquared;

    public Turret(TurretIO io) {
        this.io = io;
        turretRequest = new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS)
                - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Turret.KS;
        slot0Configs.kV = Constants.Turret.KV;
        slot0Configs.kA = Constants.Turret.KA_MM;
        slot0Configs.kP = Constants.Turret.KP;
        slot0Configs.kD = Constants.Turret.KD;

        config.ClosedLoopGeneral.ContinuousWrap = false;

        config.MotionMagic.MotionMagicExpo_kV = Constants.Turret.MMEXPO_KV;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Turret.MMEXPO_KA;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.ROTATION_MAX_POSITION_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.ROTATION_MIN_POSITION_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Turret.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(config);
        setTarget(Constants.Turret.STARTING_POSITION_RADIANS, 0, 0);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Turret/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("TURRET_FFMUL", 1.0);
    }

    @Override
    public void periodicManaged() {
        Constants.Turret.FF_MUL = SmartDashboard.getNumber("TURRET_FFMUL", 1.0);

        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysId)) {
            io.setMotionMagic(turretRequest
                    .withPosition(targetPositionRotations - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS)
                    .withIgnoreSoftwareLimits(false)
                    .withFeedForward(feedforward(
                            SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                            SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001))));
        }

        RobotContainer.model.shooterModel.updateTurret(Units.rotationsToRadians(getPositionRotations()));

        Logger.recordOutput("Turret/TargetPositionRotations", targetPositionRotations);
        Logger.recordOutput("Turret/TargetVelocityRotationsPerSecond", targetVelocityRotationsPerSecond);
        Logger.recordOutput(
                "Turret/TargetAccelerationRotationsPerSecondSquared", targetAccelerationRotationsPerSecondSquared);

        Logger.recordOutput(
                "Turret/TargetVoltage",
                feedforward(
                        SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                        SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001)));
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getPositionRotations() {
        return io.getPositionRotations() + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVelocityRotationsPerSecond() {
        return io.getVelocityRotationsPerSecond();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVoltage() {
        return io.getVoltage();
    }

    private double getSpringFeedforward() {
        double pos = io.getPositionRotations();
        return pos > Constants.Turret.TURRET_SPRING_START_POS
                ? Constants.Turret.TURRET_SPRING_VOLTS
                : pos < Constants.Turret.TURRET_SPRING_START_NEG ? -Constants.Turret.TURRET_SPRING_VOLTS : 0;
    }

    private double feedforward(double velocityRotationsPerSecond, double accelerationRotationsPerSecondSquared) {
        double velocityError = velocityRotationsPerSecond - getVelocityRotationsPerSecond();
        return Math.max(
                        -12.0,
                        Math.min(
                                12.0,
                                Constants.Turret.KV * velocityRotationsPerSecond
                                        + Constants.Turret.KA * accelerationRotationsPerSecondSquared
                                        + Constants.Turret.KS * Math.signum(velocityRotationsPerSecond)
                                        + Constants.Turret.KVP * velocityError))
                * Constants.Turret.FF_MUL;
    }

    public static double getClosestTurretPosition(double currentPos, double targetHeading) {

        double minRange = Units.rotationsToRadians(
                Constants.Turret.ROTATION_MIN_POSITION_MOTOR_ROTATIONS + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
        double maxRange = Units.rotationsToRadians(
                Constants.Turret.ROTATION_MAX_POSITION_MOTOR_ROTATIONS + MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);

        // bring target close to current revolution
        double baseTarget = targetHeading + TWO_PI * Math.round((currentPos - targetHeading) / TWO_PI);

        double bestPos = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        // check equivalent angles (target ± 2π)
        for (int i = -1; i <= 1; i++) {
            double candidate = baseTarget + i * TWO_PI;

            if (candidate >= minRange && candidate <= maxRange) {
                double dist = Math.abs(candidate - currentPos);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestPos = candidate;
                }
            }
        }

        // if none are in range, clamp to nearest limit
        if (Double.isNaN(bestPos)) {
            double clamped = Math.max(minRange, Math.min(maxRange, baseTarget));
            bestPos = clamped;
        }

        return bestPos;
    }

    public void setTarget(
            double targetPositionRadians,
            double targetVelocityRadiansPerSecond,
            double targetAccelerationRadiansPerSecondSquared) {

        this.targetPositionRotations = Units.radiansToRotations(
                getClosestTurretPosition(Units.rotationsToRadians(getPositionRotations()), targetPositionRadians));
        this.targetVelocityRotationsPerSecond = Units.radiansToRotations(targetVelocityRadiansPerSecond);
        this.targetAccelerationRotationsPerSecondSquared =
                Units.radiansToRotations(targetAccelerationRadiansPerSecondSquared);
        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysId)) {
            io.setMotionMagic(turretRequest
                    .withPosition(this.targetPositionRotations - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS)
                    .withIgnoreSoftwareLimits(false)
                    .withFeedForward(feedforward(
                            SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                            SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001))));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setVoltage(0.0);
        } else {
            io.setMotionMagic(turretRequest
                    .withPosition(this.targetPositionRotations - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS)
                    .withIgnoreSoftwareLimits(false)
                    .withFeedForward(feedforward(
                            SimpleMath.rawDeadband(targetVelocityRotationsPerSecond, 0.001),
                            SimpleMath.rawDeadband(targetAccelerationRotationsPerSecondSquared, 0.0001))));
        }
    }

    @AutoLogLevel
    public boolean atGoal() {
        return SimpleMath.isWithinTolerance(getPositionRotations(), targetPositionRotations, POSITION_TOLERANCE);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.REAL)
    public LimitSwitchStates getLimitSwitchStates() {
        return io.getLimitSwitchStates();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDrawAmps();
    }

    public void resetEncoders() {
        io.setPositionRotations(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS)
                - MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.turret.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.turret.sysIdDynamic(direction);
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
