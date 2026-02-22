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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.SysIdArm;
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

    private static final double POSITION_TOLERANCE = Units.degreesToRotations(2);
    private static final double VELOCITY_TOLERANCE = Units.degreesToRotations(50);

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(2.0).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(1.5);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.3);

    private final TurretIO io;
    private final SysIdRoutine sysIdRoutine;
    private final MotionMagicExpoVoltage turretRequest;
    private double targetPositionRotations;

    public Turret(TurretIO io) {
        this.io = io;
        turretRequest =
                new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS));

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Turret.KS;
        slot0Configs.kV = Constants.Turret.KV;
        slot0Configs.kA = Constants.Turret.KA;
        slot0Configs.kP = Constants.Turret.KP;
        slot0Configs.kD = Constants.Turret.KD;

        config.MotionMagic.MotionMagicExpo_kV = Constants.Turret.MMEXPO_KV;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Turret.MMEXPO_KA;

        config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.SUPPLY_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Turret.SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);
        config.CurrentLimits.StatorCurrentLimit = Constants.Turret.STATOR_CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(Constants.Turret.ROTATION_MAX_POSITION_RADIANS);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(Constants.Turret.ROTATION_MIN_POSITION_RADIANS);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = Constants.Turret.GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(config);
        setTarget(Constants.Turret.STARTING_POSITION_RADIANS);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Turret/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));
    }

    @Override
    public void periodicManaged() {
        RobotContainer.model.shooterModel.updateTurret(Units.rotationsToRadians(getPositionRotations()));
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getPositionRotations() {
        return io.getPositionRotations();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVelocityRotationsPerSecond() {
        return io.getVelocityRotationsPerSecond();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getVoltage() {
        return io.getVoltage();
    }

    public void setTarget(double targetPositionRadians) {
        this.targetPositionRotations = Units.radiansToRotations(targetPositionRadians);
        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysIdArm)) {
            io.setMotionMagic(turretRequest.withPosition(this.targetPositionRotations));
        }
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setVoltage(0.0);
        } else {
            io.setMotionMagic(turretRequest.withPosition(this.targetPositionRotations));
        }
    }

    public boolean atGoal() {
        return SimpleMath.isWithinTolerance(getPositionRotations(), targetPositionRotations, POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(getVelocityRotationsPerSecond(), 0, VELOCITY_TOLERANCE);
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
        io.setPositionRotations(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS));
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
