package frc.robot.utils.maplesim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayDeque;
import java.util.Queue;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class IntakeFromGoalSimulation {

    private final int capacity;
    private int gamePiecesInIntakeCount;
    private boolean intakeRunning;

    private final Queue<SourceGoal> gamePiecesToRemove;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final String targetedGamePieceType;

    public Transform3d drivetrainRelativePose = new Transform3d();

    public IntakeFromGoalSimulation(
            String targetedGamePieceType, AbstractDriveTrainSimulation driveTrainSimulation, int capacity) {

        this.targetedGamePieceType = targetedGamePieceType;
        this.gamePiecesInIntakeCount = 0;

        if (capacity > 100) throw new IllegalArgumentException("capacity too large, max is 100");
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);

        this.intakeRunning = false;
        this.driveTrainSimulation = driveTrainSimulation;

        register();
    }

    /**
     *
     *
     * <h2>Turns the Intake On.</h2>
     *
     * <p>Extends the intake out from the chassis, making it part of the chassis's collision space.
     *
     * <p>Once activated, the intake is considered running and will listen for contact with
     * {@link GamePieceOnFieldSimulation} instances, allowing it to collect game pieces.
     */
    public void startIntake() {
        if (intakeRunning) return;
        this.intakeRunning = true;
    }

    /**
     *
     *
     * <h2>Turns the Intake Off.</h2>
     *
     * <p>Retracts the intake into the chassis, removing it from the chassis's collision space.
     *
     * <p>Once turned off, the intake will no longer listen for or respond to contacts with
     * {@link GamePieceOnFieldSimulation} instances.
     */
    public void stopIntake() {
        if (!intakeRunning) return;
        this.intakeRunning = false;
    }

    /**
     *
     *
     * <h2>Get the amount of game pieces in the intake.</h2>
     *
     * @return the amount of game pieces stored in the intake
     */
    public int getGamePiecesAmount() {
        return gamePiecesInIntakeCount;
    }

    /**
     *
     *
     * <h2>Removes 1 game piece from the intake.</h2>
     *
     * <p>Deducts the {@link #getGamePiecesAmount()}} by 1, if there is any remaining.
     *
     * <p>This is used to obtain a game piece from the intake and move it a feeder/shooter.
     *
     * @return if there is game piece(s) remaining, and therefore retrieved
     */
    public boolean obtainGamePieceFromIntake() {
        if (gamePiecesInIntakeCount < 1) return false;
        gamePiecesInIntakeCount--;
        return true;
    }

    /**
     *
     *
     * <h2>Adds 1 game piece from the intake.</h2>
     *
     * <p>Increases the {@link #getGamePiecesAmount()}} by 1, if there is still space.
     *
     * @return if there is still space in the intake to perform this action
     */
    public boolean addGamePieceToIntake() {
        boolean toReturn = gamePiecesInIntakeCount < capacity;
        if (toReturn) gamePiecesInIntakeCount++;

        return toReturn;
    }

    /**
     *
     *
     * <h2>Sets the amount of game pieces in the intake.</h2>
     *
     * <p>Sets the {@link #getGamePiecesAmount()}} to a given amount.
     *
     * <p>Will make sure that the amount is non-negative and does not exceed the capacity
     *
     * @return the actual (clamped) game piece count after performing this action
     */
    public int setGamePiecesCount(int gamePiecesInIntakeCount) {
        return this.gamePiecesInIntakeCount = MathUtil.clamp(gamePiecesInIntakeCount, 0, capacity);
    }

    /**
     *
     *
     * <h2>Clears the game pieces that have been obtained by the intake.</h2>
     *
     * <p>This method is called from {@link SimulatedArena#simulationPeriodic()} to remove the
     * {@link SourceGoal} instances that have been obtained by the intake from the field.
     *
     * <p>Game pieces are marked for removal if they have come into contact with the intake during the last
     * {@link SimulatedArena#getSimulationSubTicksIn1Period()} sub-ticks. These game pieces should be removed from the
     * field to reflect their interaction with the intake.
     */
    public void removeObtainedGamePieces(SimulatedArena arena) {
        while (!gamePiecesToRemove.isEmpty()) {
            SourceGoal gamePiece = gamePiecesToRemove.poll();
            gamePiece.onIntake(this.targetedGamePieceType);
        }
    }

    public void register() {
        if (SimulatedArena.getInstance() instanceof Arena2025ReefscapeWithAlgae arena) {
            register(arena);
        }
    }

    public void register(Arena2025ReefscapeWithAlgae arena) {
        arena.addIntakeFromGoalSimulation(this);
    }

    public void checkAgainstSourceGoal(SourceGoal goal) {
        if (!intakeRunning) return;
        if (!goal.getGamePieceType().equals(targetedGamePieceType)) return;
        if (gamePiecesInIntakeCount >= capacity) return;

        Pose3d pose = new Pose3d(driveTrainSimulation.getSimulatedDriveTrainPose()).transformBy(drivetrainRelativePose);

        if (goal.checkCollision(pose)) {
            gamePiecesToRemove.add(goal);
            gamePiecesInIntakeCount++;
        }
    }

    /**
     *
     *
     * <h2>Returns wether or not this intake is currently running</h2>
     */
    public boolean isRunning() {
        return intakeRunning;
    }
}
