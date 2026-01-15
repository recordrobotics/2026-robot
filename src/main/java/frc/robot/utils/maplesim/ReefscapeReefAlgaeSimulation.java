package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class ReefscapeReefAlgaeSimulation implements SimulatedArena.Simulatable {

    protected final HashMap<String, ReefscapeReefAlgaeSide> sides;

    /**
     *
     *
     * <h2>Creates an reef of the specified color.</h2>
     *
     * @param arena The host arena of this reef.
     * @param isBlue Wether this is the blue reef or the red one.
     */
    ReefscapeReefAlgaeSimulation(Arena2025Reefscape arena, boolean isBlue) {
        sides = new HashMap<>(6);
        for (int side = 0; side < 6; side++) {
            char sideLeftChar = (char) ('A' + side);
            char sideRightChar = (char) ('A' + (side + 1));
            ReefscapeReefAlgaeSide reefSide = new ReefscapeReefAlgaeSide(arena, isBlue, side);
            sides.put(String.valueOf(sideLeftChar) + String.valueOf(sideRightChar), reefSide);
        }
    }

    public void draw(List<Pose3d> algaePosesToDisplay) {
        for (ReefscapeReefAlgaeSide side : sides.values()) {
            side.draw(algaePosesToDisplay);
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        for (ReefscapeReefAlgaeSide side : sides.values()) {
            side.simulationSubTick(subTickNum);
        }
    }

    /**
     *
     *
     * <h2>Clears the reef.</h2>
     */
    public void clearReef() {
        for (ReefscapeReefAlgaeSide side : sides.values()) {
            side.clear();
        }
    }

    /**
     *
     *
     * <h2>Resets the reef to its original state.</h2>
     */
    public void resetReef() {
        for (ReefscapeReefAlgaeSide side : sides.values()) {
            side.fill();
        }
    }

    /**
     * Obtains a specific side based on its ID.
     *
     * <p>The ID is a combination of the two branch letters (A-L)
     *
     * <p>For example, "AB" refers to the algae of branches A and B, while "CD" refers to the algae of branches C and D.
     *
     * @param id the ID of the branch to retrieve (e.g., "AB", "CD", ..., "KL")
     * @return the {@link ReefscapeReefAlgaeSide} corresponding to the given ID, or null if the ID is invalid
     */
    public ReefscapeReefAlgaeSide getSide(String id) {
        return sides.get(id);
    }

    /**
     * Obtains the set of all sides in the reef through a set of map entries (ID -> {@link ReefscapeReefAlgaeSide}).
     *
     * <p>The ID is a combination of the two branch letters (A-L)
     *
     * <p>For example, "AB" refers to the algae of branches A and B, while "CD" refers to the algae of branches C and D.
     *
     * @param id the ID of the branch to retrieve (e.g., "AB", "CD", ..., "KL")
     * @return the {@link ReefscapeReefAlgaeSide} corresponding to the given ID, or null if the ID is invalid
     */
    public Set<Map.Entry<String, ReefscapeReefAlgaeSide>> getSidesSet() {
        return sides.entrySet();
    }
}
