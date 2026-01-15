package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.SimulatedArena;

public abstract class SourceGoal extends Goal {
    /**
     *
     *
     * <h2>Creates a goal object </h2>
     *
     * @param arena The host arena of this goal
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Wether this is a blue goal or a red one.
     * @param max How many pieces can be scored in this goal.
     */
    public SourceGoal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            String gamePieceType,
            Translation3d position,
            boolean isBlue,
            int max) {
        super(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, max);
        if (arena instanceof Arena2025ReefscapeWithAlgae algaeArena) {
            algaeArena.addSourceGoal(this);
        }
    }

    /**
     *
     *
     * <h2>Creates a goal object with no scoring max.</h2>
     *
     * @param arena The host arena of this goal.
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Wether this is a blue goal or a red one.
     */
    public SourceGoal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            String gamePieceType,
            Translation3d position,
            boolean isBlue) {
        this(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, 99999);
    }

    public String getGamePieceType() {
        return this.gamePieceType;
    }

    public void onIntake(String intakeTargetGamePieceType) {}

    public abstract boolean checkCollision(Pose3d pose);
}
