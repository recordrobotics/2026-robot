package frc.robot.utils.maplesim;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import java.util.Arrays;
import java.util.List;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeReefAlgaeSide extends SourceGoal {

    public final int side;

    public static final Translation2d origin =
            new Translation2d(FieldMirroringUtils.FIELD_WIDTH / 2, FieldMirroringUtils.FIELD_HEIGHT / 2);

    public static final Translation2d blueReefCenter = origin.plus(new Translation2d(-4.2845, 0));

    private static final Translation2d localTranslation = new Translation2d(-4.969, 0).plus(origin);
    public static final Translation2d[] sidesCenterPositionBlue = new Translation2d[] {
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(0)), // AB
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(60)), // CD
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(120)), // EF
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(180)), // GH
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(240)), // IJ
        localTranslation.rotateAround(blueReefCenter, Rotation2d.fromDegrees(300)), // KL
    };

    public static final int[] heightsPerSide = new int[] {1, 0, 1, 0, 1, 0};

    public static final Translation3d[] heights =
            new Translation3d[] {new Translation3d(0, 0, 0.910), new Translation3d(0, 0, 1.315)};

    public static final Rotation3d flip90 = new Rotation3d(0, 0, Math.PI / 2);

    public static final Translation2d[] sidesCenterPositionRed = Arrays.stream(sidesCenterPositionBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Translation2d[]::new);

    public static final Rotation2d[] sidesFacingOutwardsBlue = new Rotation2d[] {
        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
        Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(240), // C and D
        Rotation2d.fromDegrees(300), Rotation2d.fromDegrees(300), // E and F
        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
        Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
        Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
    };

    /**
     *
     *
     * <h2>Returns the required pose of a reef branch at the designated position.</h2>
     *
     * @param isBlue Wether the position is on the blue reef or the red reef.
     * @param level The level of the reef (0 indexed). Range of 0-3.
     * @param col The pole or Colum of the reef (0 indexed). Range of 0-11.
     * @return The pose of a reef branch with the specified stats.
     */
    public static Translation3d getPoseOfSideAt(boolean isBlue, int side) {
        if (isBlue) {
            return new Translation3d(sidesCenterPositionBlue[side]).plus(heights[heightsPerSide[side]]);
        } else {
            return new Translation3d(sidesCenterPositionRed[side]).plus(heights[heightsPerSide[side]]);
        }
    }

    public static final Rotation2d[] sidesFacingOutwardsRed = Arrays.stream(sidesFacingOutwardsBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Rotation2d[]::new);

    /**
     *
     *
     * <h2>Creates a singular reef algae side at the specified location </h2>
     *
     * @param arena The host arena of this reef.
     * @param isBlue Wether the position is on the blue reef or the red reef.
     * @param side 0-5 the side of the reef (AB -> KL)
     */
    public ReefscapeReefAlgaeSide(Arena2025Reefscape arena, boolean isBlue, int side) {
        super(
                arena,
                Centimeters.of(30),
                Centimeters.of(30),
                Centimeters.of(30),
                "Algae",
                getPoseOfSideAt(isBlue, side),
                isBlue,
                1);

        this.side = side;
    }

    /**
     *
     *
     * <h2>Gives the pose of the reef branch.</h2>
     *
     * @return This position of this branch as a pose3d.
     */
    public Pose3d getPose() {
        return new Pose3d(
                position,
                pieceAngle != null
                        ? pieceAngle
                        : new Rotation3d(0, 0, sidesFacingOutwardsBlue[side].getRadians()).plus(flip90));
    }

    @Override
    protected void addPoints() {
        /* you can't score algae onto the reef */
    }

    public void fill() {
        gamePieceCount = 1;
    }

    @Override
    public boolean checkRotation(GamePiece gamePiece) {
        return true;
    }

    @Override
    public boolean checkCollision(GamePiece gamePiece) {
        return xyBox.contains(new Vector2(
                        gamePiece.getPose3d().getX(), gamePiece.getPose3d().getY()))
                && gamePiece.getPose3d().getZ() >= (elevation.in(Units.Meters) - 0.02)
                && gamePiece.getPose3d().getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters);
    }

    @Override
    public boolean checkCollision(Pose3d pose) {
        return xyBox.contains(new Vector2(pose.getX(), pose.getY()))
                && pose.getZ() >= (elevation.in(Units.Meters) - 0.02)
                && pose.getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters);
    }

    @Override
    public void onIntake(String intakeTargetGamePieceType) {
        if (gamePieceCount > 0) {
            gamePieceCount--;
        }
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        if (this.gamePieceCount > 0) {
            drawList.add(getPose());
        }
    }
}
