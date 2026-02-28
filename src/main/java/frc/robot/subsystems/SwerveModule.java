package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;

public final class SwerveModule implements AutoCloseable, PoweredSubsystem {

    private static final double STATIONARY_DRIVE_VELOCITY_THRESHOLD = 0.08;
    private static final double STATIONARY_TURN_VELOCITY_THRESHOLD = 1.0;
    private static final boolean USE_COSINE_COMPENSATION = false;

    private final SwerveModuleIO io;

    private double targetDriveVelocity;
    private double targetTurnPosition;

    private final double turnGearRatio;
    private final double driveGearRatio;
    private final double wheelDiameter;

    private final MotionMagicExpoVoltage turnRequest;
    private final MotionMagicVelocityVoltage driveRequest;

    private double drivePositionCached = 0;
    private double driveVelocityCached = 0;
    private double driveAccelerationCached = 0;
    private double driveVoltageCached = 0;
    private double turnPositionCached = 0;
    private double turnVelocityCached = 0;
    private double turnVoltageCached = 0;

    private double lastMovementTime = Timer.getTimestamp();
    private boolean hasResetAbs = false;

    private final Alert absEncoderDisconnectedAlert;
    private final Alert absEncoderInvalidAlert;
    private final Alert absEncoderRedAlert;
    private final Alert absEncoderOrangeAlert;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
     *
     * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
     *     module. Look at ModuleConstants.java for what variables are contained
     */
    public SwerveModule(ModuleConstants m, SwerveModuleIO io) {
        this.io = io;

        // Creates other variables
        this.turnGearRatio = m.turnGearRatio();
        this.driveGearRatio = m.driveGearRatio();
        this.wheelDiameter = m.wheelDiameter();

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsDrive = driveConfig.Slot0;
        slot0ConfigsDrive.kS = m.driveKs();
        slot0ConfigsDrive.kV = m.driveKv();
        slot0ConfigsDrive.kA = m.driveKa();
        slot0ConfigsDrive.kP = m.driveKp();
        slot0ConfigsDrive.kI = 0;
        slot0ConfigsDrive.kD = 0;

        double wheelCircumference = wheelDiameter * Math.PI;
        driveConfig.Feedback.SensorToMechanismRatio = driveGearRatio / wheelCircumference;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsDrive = driveConfig.MotionMagic;
        motionMagicConfigsDrive.MotionMagicAcceleration = Constants.Swerve.DRIVE_MAX_ACCELERATION;
        motionMagicConfigsDrive.MotionMagicJerk = Constants.Swerve.DRIVE_MAX_JERK;

        io.applyDriveTalonFXConfig(driveConfig
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(m.driveMotorSupplyCurrentLimit())
                        .withStatorCurrentLimit(m.driveMotorStatorCurrentLimit())
                        .withSupplyCurrentLowerLimit(m.driveMotorSupplyLowerCurrentLimit())
                        .withSupplyCurrentLowerTime(m.driveMotorSupplyLowerCurrentLimitTime())
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsTurn = turnConfig.Slot0;
        slot0ConfigsTurn.kS = m.turnKs();
        slot0ConfigsTurn.kV = m.turnKv();
        slot0ConfigsTurn.kA = m.turnKa();
        slot0ConfigsTurn.kP = m.turnKp();
        slot0ConfigsTurn.kI = 0;
        slot0ConfigsTurn.kD = m.turnKd();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.Feedback.SensorToMechanismRatio = turnGearRatio;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsTurn = turnConfig.MotionMagic;
        motionMagicConfigsTurn.MotionMagicExpo_kV = Constants.Swerve.TURN_MMEXPO_KV;
        motionMagicConfigsTurn.MotionMagicExpo_kA = Constants.Swerve.TURN_MMEXPO_KA;

        io.applyTurnTalonFXConfig(turnConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(m.turnMotorSupplyCurrentLimit())
                        .withStatorCurrentLimit(m.turnMotorStatorCurrentLimit())
                        .withSupplyCurrentLowerLimit(m.turnMotorSupplyLowerCurrentLimit())
                        .withSupplyCurrentLowerTime(m.turnMotorSupplyLowerCurrentLimitTime())
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        // Sets motor speeds to 0
        io.setDriveMotorVoltage(0);
        io.setTurnMotorVoltage(0);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = m.turningEncoderOffset();

        io.applyTurningEncoderConfig(encoderConfig);

        // Corrects for offset in absolute wheel position
        if (isAbsEncoderConnected()) {
            turnPositionCached = io.getAbsoluteEncoder();
            io.setTurnMechanismPosition(turnPositionCached);
        } else {
            turnPositionCached = io.getTurnMechanismPosition();
        }

        absEncoderDisconnectedAlert = new Alert(
                "Absolute encoder " + m.absoluteTurningMotorEncoderChannel() + " disconnected", Alert.AlertType.kError);
        absEncoderInvalidAlert = new Alert(
                "Absolute encoder " + m.absoluteTurningMotorEncoderChannel() + " invalid magnet health",
                Alert.AlertType.kError);
        absEncoderRedAlert = new Alert(
                "Absolute encoder " + m.absoluteTurningMotorEncoderChannel() + " red magnet health",
                Alert.AlertType.kWarning);
        absEncoderOrangeAlert = new Alert(
                "Absolute encoder " + m.absoluteTurningMotorEncoderChannel() + " orange magnet health",
                Alert.AlertType.kWarning);

        updateAbsEncoderAlert();

        turnRequest = new MotionMagicExpoVoltage(turnPositionCached);
        driveRequest = new MotionMagicVelocityVoltage(0);
    }

    private void updateAbsEncoderAlert() {
        absEncoderDisconnectedAlert.set(!isAbsEncoderConnected());
        absEncoderInvalidAlert.set(getAbsEncoderMagnetHealth() == MagnetHealthValue.Magnet_Invalid);
        absEncoderRedAlert.set(getAbsEncoderMagnetHealth() == MagnetHealthValue.Magnet_Red);
        absEncoderOrangeAlert.set(getAbsEncoderMagnetHealth() == MagnetHealthValue.Magnet_Orange);
    }

    public boolean isAbsEncoderConnected() {
        return io.isAbsEncoderConnected();
    }

    public MagnetHealthValue getAbsEncoderMagnetHealth() {
        return io.getAbsEncoderMagnetHealth();
    }

    /**
     * Returns the current rotation of the swerve wheel as a Rotation2d object.
     * @return The raw rotations of the turning motor (rotation 2d object).
     */
    public Rotation2d getTurnWheelRotation2d() {
        // Get the wheel's current turn position in rotations
        double numWheelRotations = turnPositionCached;
        // Convert wheel rotations to radians
        double wheelRadians = numWheelRotations * SimpleMath.PI2;
        // Create a Rotation2d object from the wheel's angle in radians
        return new Rotation2d(wheelRadians);
    }

    public double getTurnWheelVelocity() {
        return turnVelocityCached;
    }

    // meters per second
    public double getDriveWheelVelocity() {
        return driveVelocityCached;
    }

    // meters per second^2
    public double getDriveWheelAcceleration() {
        return driveAccelerationCached;
    }

    // meters
    public double getDriveWheelDistance() {
        return drivePositionCached;
    }

    /**
     * *custom function
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveWheelVelocity(), getTurnWheelRotation2d());
    }

    public SwerveModuleState getModuleStateAcceleration() {
        return new SwerveModuleState(getDriveWheelAcceleration(), getTurnWheelRotation2d());
    }

    /**
     * *custom function
     *
     * @return The current position of the module as a SwerveModulePosition object.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveWheelDistance(), getTurnWheelRotation2d());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(getTurnWheelRotation2d());

        targetTurnPosition = desiredState.angle.getRotations();
        targetDriveVelocity = desiredState.speedMetersPerSecond;
    }

    public void periodic() {
        drivePositionCached = io.getDriveMechanismPosition();
        driveVelocityCached = io.getDriveMechanismVelocity();
        driveAccelerationCached = io.getDriveMechanismAcceleration();
        turnPositionCached = io.getTurnMechanismPosition();
        turnVelocityCached = io.getTurnMechanismVelocity();
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.SYSID)) {
            driveVoltageCached = io.getDriveMotorVoltage();
            turnVoltageCached = io.getTurnMotorVoltage();
        }

        updateAbsEncoderAlert();

        if (Math.abs(driveVelocityCached) > STATIONARY_DRIVE_VELOCITY_THRESHOLD
                || Math.abs(turnVelocityCached) > STATIONARY_TURN_VELOCITY_THRESHOLD) {
            hasResetAbs = false;
            lastMovementTime = Timer.getTimestamp();
        } else if (Timer.getTimestamp() - lastMovementTime > 2.0
                && isAbsEncoderConnected()
                && !hasResetAbs) { // if still for 2 seconds
            hasResetAbs = true;
            turnPositionCached = io.getAbsoluteEncoder();
            io.setTurnMechanismPosition(turnPositionCached);
        }

        double actualTargetDriveVelocity = targetDriveVelocity;
        if (USE_COSINE_COMPENSATION) {
            actualTargetDriveVelocity *= Math.cos(Units.rotationsToRadians(targetTurnPosition)
                    - getTurnWheelRotation2d().getRadians());
        }

        io.setDriveMotorMotionMagic(driveRequest.withVelocity(actualTargetDriveVelocity));

        if (!(SysIdManager.getProvider() instanceof Drivetrain.SysIdTurn)) {
            io.setTurnMotorMotionMagic(turnRequest.withPosition(targetTurnPosition));
        }
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void stop() {
        targetDriveVelocity = 0;
        io.setDriveMotorVoltage(0);
        io.setTurnMotorVoltage(0);
    }

    public void setDriveMotorVoltsSysIdOnly(double volts) {
        io.setDriveMotorVoltage(volts);
    }

    public double getDriveMotorVoltsSysIdOnly() {
        return driveVoltageCached;
    }

    public void setTurnMotorVoltsSysIdOnly(double volts) {
        io.setTurnMotorVoltage(volts);
    }

    public double getTurnMotorVoltsSysIdOnly() {
        return turnVoltageCached;
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getDriveMotorCurrentDrawAmps() + io.getTurnMotorCurrentDrawAmps();
    }
}
