package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.TurretIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;

public final class Turret extends KillableSubsystem implements PoweredSubsystem {

    private static final double POSITION_TOLERANCE = Units.degreesToRotations(2);

    private final TurretIO io;
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
        slot0Configs.kI = 0;
        slot0Configs.kD = Constants.Turret.KD;

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
        io.setMotionMagic(turretRequest.withPosition(this.targetPositionRotations));
    }

    public boolean atGoal() {
        return SimpleMath.isWithinTolerance(getPositionRotations(), targetPositionRotations, POSITION_TOLERANCE);
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDrawAmps();
    }

    public void resetEncoders() {
        io.setPositionRotations(Units.radiansToRotations(Constants.Turret.STARTING_POSITION_RADIANS));
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    @Override
    public void kill() {
        io.setVoltage(0);
    }
}
