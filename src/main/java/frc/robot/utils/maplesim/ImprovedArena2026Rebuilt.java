package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.utils.mathutils.GeometryConvertor;

public class ImprovedArena2026Rebuilt extends Arena2026Rebuilt {

    public ImprovedArena2026Rebuilt(boolean addRampColliders) {
        super(addRampColliders);
        super.addCustomSimulation(ImprovedRebuiltFuelOnField::updateAll);
        super.addCustomSimulation(ImprovedRebuiltFuelOnFly::updateAll);
    }

    // Intercept game pieces to wrap them around the ImprovedRebuiltFuelOnField class
    @Override
    public synchronized void addGamePiece(GamePieceOnFieldSimulation gamePiece) {
        Pose2d initialPose = gamePiece.getPoseOnField();
        Translation2d initialVelocityMPS = GeometryConvertor.toWpilibTranslation2d(gamePiece.getLinearVelocity());

        ImprovedRebuiltFuelOnField wrappedGamePiece = new ImprovedRebuiltFuelOnField(initialPose, initialVelocityMPS);

        super.addGamePiece(wrappedGamePiece);
    }

    @Override
    public synchronized void addGamePieceProjectile(GamePieceProjectile gamePieceProjectile) {
        Pose3d initialPose = gamePieceProjectile.getPose3d();
        Translation3d initialVelocityMPS = gamePieceProjectile.getVelocity3dMPS();

        ImprovedRebuiltFuelOnFly wrappedGamePieceProjectile = new ImprovedRebuiltFuelOnFly(
                initialPose.getTranslation().toTranslation2d(),
                initialVelocityMPS.toTranslation2d(),
                initialPose.getTranslation().getZ(),
                initialVelocityMPS.getZ(),
                initialPose.getRotation());

        super.addGamePieceProjectile(wrappedGamePieceProjectile);
    }
}
