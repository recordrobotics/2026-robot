package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.subsystems.io.ElevatorIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public final class Elevator extends ManagedSubsystemBase implements PoweredSubsystem, EncoderResettableSubsystem {

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(4.5).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(3.0);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.2);

    private final ElevatorIO io;

    private final MotionMagicExpoVoltage elevatorRequest;
    private final Follower elevatorFollower;

    private double leadPositionCached = 0;
    private double leadVelocityCached = 0;
    private double leadVoltageCached = 0;

    private double setpoint;

    private final SysIdRoutine sysIdRoutine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsElevator = elevatorConfig.Slot0;
        slot0ConfigsElevator.kS = Constants.Elevator.KS;
        slot0ConfigsElevator.kV = Constants.Elevator.KV;
        slot0ConfigsElevator.kA = Constants.Elevator.KA;
        slot0ConfigsElevator.kG = Constants.Elevator.KG;
        slot0ConfigsElevator.kP = Constants.Elevator.KP;
        slot0ConfigsElevator.kI = 0;
        slot0ConfigsElevator.kD = Constants.Elevator.KD;
        slot0ConfigsElevator.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Feedback.SensorToMechanismRatio = 1.0 / Constants.Elevator.METERS_PER_ROTATION;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsElevator = elevatorConfig.MotionMagic;
        motionMagicConfigsElevator.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY;
        motionMagicConfigsElevator.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION;
        motionMagicConfigsElevator.MotionMagicJerk = Constants.Elevator.MAX_JERK;
        motionMagicConfigsElevator.MotionMagicExpo_kV = Constants.Elevator.MMEXPO_KV;
        motionMagicConfigsElevator.MotionMagicExpo_kA = Constants.Elevator.MMEXPO_KA;

        io.applyTalonFXConfig(elevatorConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Elevator.SUPPLY_CURRENT_LIMIT)
                        .withSupplyCurrentLowerLimit(Constants.Elevator.SUPPLY_CURRENT_LOWER_LIMIT)
                        .withSupplyCurrentLowerTime(1)
                        .withStatorCurrentLimit(Constants.Elevator.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        leadPositionCached = io.getLeadMotorPosition();

        elevatorRequest = new MotionMagicExpoVoltage(Constants.Elevator.STARTING_HEIGHT);
        elevatorFollower = io.createFollower();

        io.setFollowerMotionMagic(elevatorFollower);

        set(Constants.Elevator.STARTING_HEIGHT);

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Elevator/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setLeadMotorVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("Elevator", Constants.Elevator.STARTING_HEIGHT);
    }

    /** Height of the elevator in meters */
    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentHeight() {
        return leadPositionCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentVelocity() {
        return leadVelocityCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getCurrentVoltage() {
        return leadVoltageCached;
    }

    @AutoLogLevel(level = Level.DEBUG_REAL)
    public boolean isBottomEndStopPressed() {
        return io.isBottomEndStopPressed();
    }

    @AutoLogLevel(level = Level.DEBUG_REAL)
    public boolean isTopEndStopPressed() {
        return io.isTopEndStopPressed();
    }

    @Override
    public void periodicManaged() {

        leadPositionCached = io.getLeadMotorPosition();
        leadVelocityCached = io.getLeadMotorVelocity();
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.SYSID)) {
            leadVoltageCached = io.getLeadMotorVoltage();
        }

        // Update mechanism
        RobotContainer.model.elevator.update(getCurrentHeight());
        RobotContainer.model.elevator.updateSetpoint(setpoint);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public void set(double heightMeters) {
        setpoint = heightMeters;

        if (!(SysIdManager.getProvider() instanceof SysId)) {
            io.setLeadMotionMagic(elevatorRequest.withPosition(heightMeters));
        }
    }

    public void moveTo(ElevatorHeight height) {
        set(height.getHeight());
    }

    @AutoLogLevel(level = Level.REAL)
    public ElevatorHeight getNearestHeight() {
        double currentHeight = getCurrentHeight();
        double currentArmAngle = RobotContainer.elevatorArm.getArmAngle();

        ElevatorHeight[] heights = ElevatorHeight.values();
        Arrays.sort(
                heights,
                (a, b) -> Double.compare(
                        a.getDifference(currentHeight, currentArmAngle),
                        b.getDifference(currentHeight, currentArmAngle)));
        return heights[0];
    }

    public boolean atGoal() {
        return atGoal(Constants.Elevator.AT_GOAL_POSITION_TOLERANCE, Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE);
    }

    public boolean atGoal(double positionThresholdMeters, double velocityThresholdMetersPerSecond) {
        return Math.abs(setpoint - getCurrentHeight()) < positionThresholdMeters
                && Math.abs(getCurrentVelocity()) < velocityThresholdMetersPerSecond;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getLeadMotorCurrentDraw() + io.getFollowerMotorCurrentDraw();
    }

    @Override
    public void resetEncoders() {
        io.setLeadMotorPosition(Constants.Elevator.STARTING_HEIGHT);
        io.setFollowerMotorPosition(Constants.Elevator.STARTING_HEIGHT);

        leadPositionCached = Constants.Elevator.STARTING_HEIGHT;
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.elevator.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.elevator.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
