package frc.robot.utils.maplesim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import java.lang.reflect.Field;
import java.util.WeakHashMap;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ImprovedRebuiltFuelOnFly extends RebuiltFuelOnFly {

    private static final double GRAVITY = 9.81;
    private static final double AIR_DENSITY = 1.225;
    private static final double BALL_MASS = 0.218;
    private static final double BALL_RADIUS = 0.15 / 2;
    private static final double DRAG_COEFF = 0.15625;
    private static final double LIFT_COEFF = 1.399844;
    private static final double CROSS_SECTIONAL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;

    // Precomputed constants for faster simulation
    private static final double DRAG_K = 0.5 * AIR_DENSITY * DRAG_COEFF * CROSS_SECTIONAL_AREA;
    private static final double MAGNUS_K = 0.5 * AIR_DENSITY * CROSS_SECTIONAL_AREA * BALL_RADIUS * LIFT_COEFF;
    private static final double INV_MASS = 1.0 / BALL_MASS; // multiplication faster

    private static final WeakHashMap<ImprovedRebuiltFuelOnFly, ImprovedRebuiltFuelOnFly> instances =
            new WeakHashMap<>();

    private static final Field initialPositionField;
    private static final Field initialLaunchingVelocityMPSField;
    private static final Field initialHeightField;
    private static final Field initialVerticalSpeedMPSField;
    private static final Field gamePieceRotationField;

    private Vector<N3> spinAxis = Translation3d.kZero.toVector();

    private RK4State currentRK4State;

    static {
        try {
            initialPositionField = GamePieceProjectile.class.getDeclaredField("initialPosition");
            initialPositionField.setAccessible(true);

            initialLaunchingVelocityMPSField =
                    GamePieceProjectile.class.getDeclaredField("initialLaunchingVelocityMPS");
            initialLaunchingVelocityMPSField.setAccessible(true);

            initialHeightField = GamePieceProjectile.class.getDeclaredField("initialHeight");
            initialHeightField.setAccessible(true);

            initialVerticalSpeedMPSField = GamePieceProjectile.class.getDeclaredField("initialVerticalSpeedMPS");
            initialVerticalSpeedMPSField.setAccessible(true);

            gamePieceRotationField = GamePieceProjectile.class.getDeclaredField("gamePieceRotation");
            gamePieceRotationField.setAccessible(true);

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    public ImprovedRebuiltFuelOnFly(
            Translation2d initialPosition,
            Translation2d initialLaunchingVelocityMPS,
            double initialHeight,
            double initialVerticalSpeedMPS,
            Rotation3d gamePieceRotation) {
        super(
                Translation2d.kZero,
                Translation2d.kZero,
                new ChassisSpeeds(),
                Rotation2d.kZero,
                Meters.of(0),
                MetersPerSecond.of(0),
                Degrees.of(0));

        try {
            initialPositionField.set(this, initialPosition);
            initialLaunchingVelocityMPSField.set(this, initialLaunchingVelocityMPS);
            initialHeightField.set(this, initialHeight);
            initialVerticalSpeedMPSField.set(this, initialVerticalSpeedMPS);
            gamePieceRotationField.set(this, gamePieceRotation);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        spinAxis = gamePieceRotation.getAxis();

        currentRK4State = new RK4State(
                initialPosition.getX(),
                initialPosition.getY(),
                initialHeight,
                gamePieceRotation.getAngle(),
                initialLaunchingVelocityMPS.getX(),
                initialLaunchingVelocityMPS.getY(),
                initialVerticalSpeedMPS,
                0.0);
        instances.put(this, this);
    }

    public void setSpin(Vector<N3> spinAxis, double spinRadiansPerSecond) {
        this.spinAxis = spinAxis;
        currentRK4State.omega = spinRadiansPerSecond;
    }

    @Override
    protected Translation3d getPositionAtTime(double t) {
        return new Translation3d(currentRK4State.x, currentRK4State.y, currentRK4State.z);
    }

    @Override
    public Pose3d getPose3d() {
        return new Pose3d(
                currentRK4State.x,
                currentRK4State.y,
                currentRK4State.z,
                new Rotation3d(spinAxis, currentRK4State.theta));
    }

    private static final class RK4State {
        double x, y, z;
        double theta;
        double vx, vy, vz;
        double omega;

        public RK4State() {
            this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public RK4State(double x, double y, double z, double theta, double vx, double vy, double vz, double omega) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.theta = theta;
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
            this.omega = omega;
        }

        public void updateWith(RK4State other, double dt, RK4State derivative) {
            this.x = other.x + dt * derivative.x;
            this.y = other.y + dt * derivative.y;
            this.z = other.z + dt * derivative.z;
            this.theta = other.theta + dt * derivative.theta;
            this.vx = other.vx + dt * derivative.vx;
            this.vy = other.vy + dt * derivative.vy;
            this.vz = other.vz + dt * derivative.vz;
            this.omega = other.omega + dt * derivative.omega;
        }

        public void updateFromRK4(RK4State k1, RK4State k2, RK4State k3, RK4State k4, double dt) {
            double sixthDt = dt / 6.0;

            this.x += sixthDt * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
            this.y += sixthDt * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
            this.z += sixthDt * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z);
            this.theta += sixthDt * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
            this.vx += sixthDt * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx);
            this.vy += sixthDt * (k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy);
            this.vz += sixthDt * (k1.vz + 2.0 * k2.vz + 2.0 * k3.vz + k4.vz);
            this.omega += sixthDt * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega);
        }
    }

    private static void evaluateRK4VectorField(RK4State state, RK4State derivative, Vector<N3> spinAxis) {
        double speedSq = state.vx * state.vx + state.vy * state.vy + state.vz * state.vz;

        double dragFx = 0.0;
        double dragFy = 0.0;
        double dragFz = 0.0;

        if (speedSq > 0.0) {
            double speed = Math.sqrt(speedSq);

            double dragFactor = -DRAG_K * speed;

            dragFx = dragFactor * state.vx;
            dragFy = dragFactor * state.vy;
            dragFz = dragFactor * state.vz;
        }

        // 3D angular velocity vector
        double omegaX = state.omega * spinAxis.get(0);
        double omegaY = state.omega * spinAxis.get(1);
        double omegaZ = state.omega * spinAxis.get(2);

        // Cross product of omega and v
        double magnusFx = MAGNUS_K * (omegaY * state.vz - omegaZ * state.vy);
        double magnusFy = MAGNUS_K * (omegaZ * state.vx - omegaX * state.vz);
        double magnusFz = MAGNUS_K * (omegaX * state.vy - omegaY * state.vx);

        // Only forces acting on ball are drag, magnus, and gravity (z axis)
        // Divide by mass b/c F=ma, and a = F/m
        double ax = (dragFx + magnusFx) * INV_MASS;
        double ay = (dragFy + magnusFy) * INV_MASS;
        double az = (dragFz + magnusFz - BALL_MASS * GRAVITY) * INV_MASS;

        // Populate derivatives

        // Position derivatives are just the current velocity
        derivative.x = state.vx;
        derivative.y = state.vy;
        derivative.z = state.vz;
        derivative.theta = state.omega;

        // Velocity derivatives are the accelerations from forces (calculated)
        derivative.vx = ax;
        derivative.vy = ay;
        derivative.vz = az;

        // don't simulate spin decay (measured to not be significant during tracking)
        derivative.omega = 0.0;
    }

    private static final RK4State k1 = new RK4State();
    private static final RK4State k2 = new RK4State();
    private static final RK4State k3 = new RK4State();
    private static final RK4State k4 = new RK4State();
    private static final RK4State temp = new RK4State();

    private static void rk4Step(RK4State state, double dt, Vector<N3> spinAxis) {
        double halfDt = 0.5 * dt;

        evaluateRK4VectorField(state, k1, spinAxis);
        temp.updateWith(state, halfDt, k1);
        evaluateRK4VectorField(temp, k2, spinAxis);
        temp.updateWith(state, halfDt, k2);
        evaluateRK4VectorField(temp, k3, spinAxis);
        temp.updateWith(state, dt, k3);
        evaluateRK4VectorField(temp, k4, spinAxis);

        state.updateFromRK4(k1, k2, k3, k4, dt);
    }

    public void update(int subTickNum) {
        // update rk4 state with sub tick dt
        rk4Step(currentRK4State, SimulatedArena.getSimulationDt().in(Seconds), spinAxis);
    }

    public static void updateAll(int subTickNum) {
        for (ImprovedRebuiltFuelOnFly fuel : instances.keySet()) {
            fuel.update(subTickNum);
        }
    }
}
