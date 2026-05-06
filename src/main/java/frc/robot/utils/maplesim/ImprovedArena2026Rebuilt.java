package frc.robot.utils.maplesim;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class ImprovedArena2026Rebuilt extends Arena2026Rebuilt {

    public ImprovedArena2026Rebuilt(boolean addRampColliders) {
        super(addRampColliders);

        super.addCustomSimulation(ImprovedRebuiltFuelOnField::updateAll);
    }

    @Override
    public void placeGamePiecesOnField() {
        blueOutpost.reset();
        redOutpost.reset();

        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
                addGamePiece(new ImprovedRebuiltFuelOnField(centerPieceBottomRightCorner.plus(
                        new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
            }
        }

        boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
                && DriverStation.getAlliance().get() == Alliance.Blue;

        if (isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new ImprovedRebuiltFuelOnField(blueDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new ImprovedRebuiltFuelOnField(redDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }
}
