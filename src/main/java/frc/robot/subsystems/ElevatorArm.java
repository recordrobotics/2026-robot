package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ElevatorArmIO;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class ElevatorArm extends ManagedSubsystemBase implements PoweredSubsystem, EncoderResettableSubsystem {

    public static final double POSITION_TOLERANCE = 0.15;
    public static final double VELOCITY_TOLERANCE = 1.05;

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(2.0).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(1.5);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.3);

    private final ElevatorArmIO io;
    private final SysIdRoutine sysIdRoutine;

    private final MotionMagicExpoVoltage armRequest;

    private double positionCached = 0;
    private double velocityCached = 0;
    private double voltageCached = 0;

    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

    public ElevatorArm(ElevatorArmIO io) {
        this.io = io;

        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsArm = armConfig.Slot0;
        slot0ConfigsArm.kS = Constants.ElevatorArm.KS;
        slot0ConfigsArm.kV = Constants.ElevatorArm.KV;
        slot0ConfigsArm.kA = Constants.ElevatorArm.KA;
        slot0ConfigsArm.kG = Constants.ElevatorArm.KG;
        slot0ConfigsArm.kP = Constants.ElevatorArm.KP;
        slot0ConfigsArm.kI = 0;
        slot0ConfigsArm.kD = Constants.ElevatorArm.KD;
        slot0ConfigsArm.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Feedback.SensorToMechanismRatio = Constants.ElevatorArm.ARM_GEAR_RATIO;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsArm = armConfig.MotionMagic;
        motionMagicConfigsArm.MotionMagicCruiseVelocity = Constants.ElevatorArm.MAX_ARM_VELOCITY;
        motionMagicConfigsArm.MotionMagicAcceleration = Constants.ElevatorArm.MAX_ARM_ACCELERATION;
        motionMagicConfigsArm.MotionMagicJerk = Constants.ElevatorArm.MAX_JERK;
        motionMagicConfigsArm.MotionMagicExpo_kV = Constants.ElevatorArm.MMEXPO_KV;
        motionMagicConfigsArm.MotionMagicExpo_kA = Constants.ElevatorArm.MMEXPO_KA;

        io.applyArmTalonFXConfig(armConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ElevatorArm.ARM_SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(Constants.ElevatorArm.ARM_STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        positionCached = io.getArmPosition();

        armRequest = new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.ElevatorArm.START_POS));
        set(ElevatorHeight.BOTTOM.getArmAngle());

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("ElevatorArm/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setArmVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("ElevatorArm", Constants.ElevatorArm.START_POS);
    }

    public ElevatorArmSim getSimIO() throws IllegalStateException {
        if (io instanceof ElevatorArmSim simIO) {
            return simIO;
        } else {
            throw new IllegalStateException("ElevatorArmIO is not a simulation");
        }
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmAngle() {
        return positionCached * SimpleMath.PI2;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmVelocity() {
        return velocityCached * SimpleMath.PI2;
    }

    /** Used for sysid as units have to be in rotations in the logs */
    @AutoLogLevel(level = Level.SYSID)
    public double getArmAngleRotations() {
        return positionCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmVelocityRotations() {
        return velocityCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmSetTo() {
        return voltageCached;
    }

    public void set(double angleRadians) {
        currentSetpoint.position = angleRadians;
        if (!(SysIdManager.getProvider() instanceof SysId)) {
            io.setArmMotionMagic(armRequest.withPosition(Units.radiansToRotations(angleRadians)));
        }
    }

    public boolean atGoal() {
        return atGoal(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }

    public boolean atGoal(double angleThresholdRadians, double velocityThresholdRadiansPerSecond) {
        return SimpleMath.isWithinTolerance(getArmAngle(), currentSetpoint.position, angleThresholdRadians)
                && SimpleMath.isWithinTolerance(getArmVelocity(), 0, velocityThresholdRadiansPerSecond);
    }

    @Override
    public void periodicManaged() {

        positionCached = io.getArmPosition();
        velocityCached = io.getArmVelocity();
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.SYSID)) {
            voltageCached = io.getArmVoltage();
        }

        // Update mechanism
        RobotContainer.model.elevatorArm.update(getArmAngle());
        RobotContainer.model.elevatorArm.updateSetpoint(currentSetpoint.position);
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
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getArmCurrentDrawAmps();
    }

    @Override
    public void resetEncoders() {
        io.setArmPosition(Units.radiansToRotations(Constants.ElevatorArm.START_POS));
        positionCached = Units.radiansToRotations(Constants.ElevatorArm.START_POS);
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.elevatorArm.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.elevatorArm.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
