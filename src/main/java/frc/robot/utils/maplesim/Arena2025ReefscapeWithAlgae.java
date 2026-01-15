package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class Arena2025ReefscapeWithAlgae extends Arena2025Reefscape {

    public final ReefscapeReefAlgaeSimulation redAlgaeSimulation;
    public final ReefscapeReefAlgaeSimulation blueAlgaeSimulation;

    private final List<IntakeFromGoalSimulation> intakeFromGoalSimulations;
    private final List<SourceGoal> sourceGoals;

    public Arena2025ReefscapeWithAlgae() {
        super();

        this.intakeFromGoalSimulations = new ArrayList<>();
        this.sourceGoals = new ArrayList<>();

        redAlgaeSimulation = new ReefscapeReefAlgaeSimulation(this, false);
        super.addCustomSimulation(redAlgaeSimulation);

        blueAlgaeSimulation = new ReefscapeReefAlgaeSimulation(this, true);
        super.addCustomSimulation(blueAlgaeSimulation);
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        redAlgaeSimulation.clearReef();
        blueAlgaeSimulation.clearReef();
    }

    @Override
    public synchronized void placeGamePiecesOnField() {
        super.placeGamePiecesOnField();
        redAlgaeSimulation.resetReef();
        blueAlgaeSimulation.resetReef();
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        // add algae
        if (type.equals("Algae")) {
            redAlgaeSimulation.draw(poses);
            blueAlgaeSimulation.draw(poses);
        }

        return poses;
    }

    /**
     *
     *
     * <h2>Registers an {@link IntakeFromGoalSimulation}.</h2>
     *
     * <p><strong>NOTE:</strong> This method is automatically called in the constructor of {@link IntakeSimulation}, so
     * you don't need to call it manually.
     *
     * <p>The intake simulation should be bound to an {@link AbstractDriveTrainSimulation} and becomes part of its
     * collision space.
     *
     * <p>This method immediately starts the {@link org.ironmaple.simulation.IntakeSimulation.GamePieceContactListener},
     * which listens for contact between the intake and any game piece.
     *
     * @param intakeSimulation the intake simulation to be registered
     */
    protected synchronized void addIntakeFromGoalSimulation(IntakeFromGoalSimulation intakeSimulation) {
        this.intakeFromGoalSimulations.add(intakeSimulation);
    }

    protected synchronized void addSourceGoal(SourceGoal goal) {
        this.sourceGoals.add(goal);
    }

    @Override
    protected void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);

        for (IntakeFromGoalSimulation intakeSimulation : intakeFromGoalSimulations) {
            for (SourceGoal sourceGoal : sourceGoals) {
                intakeSimulation.checkAgainstSourceGoal(sourceGoal);
            }

            intakeSimulation.removeObtainedGamePieces(this);
        }
    }
}
