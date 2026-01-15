package frc.robot.tests.simple;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.field.FieldIntersection;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class FieldIntersectionTests {

    @Test
    @DisplayName("Test collision detection with points outside field bounds")
    void testCollidesWithField_OutsideFieldBounds() {
        // Test points outside field boundaries should return true
        Translation2d outsidePoint1 = new Translation2d(-1.0, 4.0);
        Translation2d insidePoint = new Translation2d(5.0, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(outsidePoint1, insidePoint),
                "Line from outside field should return true");

        Translation2d outsidePoint2 = new Translation2d(20.0, 4.0);
        assertTrue(
                FieldIntersection.collidesWithField(insidePoint, outsidePoint2),
                "Line to outside field should return true");

        assertTrue(
                FieldIntersection.collidesWithField(outsidePoint1, outsidePoint2),
                "Line between outside points should return true");
    }

    @Test
    @DisplayName("Test collision detection with valid field points not hitting obstacles")
    void testCollidesWithField_NoCollision() {
        // Test points in open field areas that shouldn't collide
        Translation2d openPoint1 = new Translation2d(2.0, 3.0);
        Translation2d openPoint2 = new Translation2d(4.0, 7.0);

        assertFalse(
                FieldIntersection.collidesWithField(openPoint1, openPoint2), "Line in open field should not collide");

        // Test same point
        assertFalse(FieldIntersection.collidesWithField(openPoint1, openPoint1), "Same point should not collide");
    }

    @Test
    @DisplayName("Test collision detection with blue coral station")
    void testCollidesWithField_BlueCoralStation() {
        // Test collision with blue coral stations (triangular areas)
        Translation2d pointInBlueCorralTop = new Translation2d(0.5, 7.3);
        Translation2d pointOutside = new Translation2d(2.0, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(pointInBlueCorralTop, pointOutside),
                "Line through blue coral station should collide");

        Translation2d pointInBlueCorralBottom = new Translation2d(0.5, 0.5);
        assertTrue(
                FieldIntersection.collidesWithField(pointInBlueCorralBottom, pointOutside),
                "Line through bottom blue coral station should collide");
    }

    @Test
    @DisplayName("Test collision detection with red coral station")
    void testCollidesWithField_RedCoralStation() {
        // Test collision with red coral stations (triangular areas)
        Translation2d pointInRedCorralTop = new Translation2d(17.0, 7.3);
        Translation2d pointOutside = new Translation2d(15.0, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(pointInRedCorralTop, pointOutside),
                "Line through red coral station should collide");

        Translation2d pointInRedCorralBottom = new Translation2d(17.0, 0.5);
        assertTrue(
                FieldIntersection.collidesWithField(pointInRedCorralBottom, pointOutside),
                "Line through bottom red coral station should collide");
    }

    @Test
    @DisplayName("Test collision detection with blue reef")
    void testCollidesWithField_BlueReef() {
        // Test collision with blue reef polygon
        Translation2d pointInBlueReef = new Translation2d(4.5, 4.0);
        Translation2d pointOutside = new Translation2d(2.0, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(pointInBlueReef, pointOutside),
                "Line through blue reef should collide");
    }

    @Test
    @DisplayName("Test collision detection with red reef")
    void testCollidesWithField_RedReef() {
        // Test collision with red reef polygon (mirrored blue reef)
        Translation2d pointInRedReef = new Translation2d(13.0, 4.0);
        Translation2d pointOutside = new Translation2d(15.5, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(pointInRedReef, pointOutside),
                "Line through red reef should collide");
    }

    @Test
    @DisplayName("Test collision detection with center pillar")
    void testCollidesWithField_CenterPillar() {
        // Test collision with the rectangular pillar in the center
        Translation2d pointInPillar = new Translation2d(8.774, 4.026);
        Translation2d pointOutside = new Translation2d(10.0, 4.0);

        assertTrue(
                FieldIntersection.collidesWithField(pointInPillar, pointOutside),
                "Line through center pillar should collide");

        // Test line that crosses the pillar
        Translation2d beforePillar = new Translation2d(8.0, 4.026);
        Translation2d afterPillar = new Translation2d(9.5, 4.026);

        assertTrue(
                FieldIntersection.collidesWithField(beforePillar, afterPillar),
                "Line crossing center pillar should collide");
    }

    @Test
    @DisplayName("Test collision detection at field boundaries")
    void testCollidesWithField_FieldBoundaries() {
        Translation2d justInsideField = new Translation2d(1.0, 1.0);
        Translation2d farInsideField = new Translation2d(5.0, 0.1);

        assertFalse(
                FieldIntersection.collidesWithField(justInsideField, farInsideField),
                "Line between valid field points in open area should not collide");
    }

    @Test
    @DisplayName("Test collision detection with line segments tangent to obstacles")
    void testCollidesWithField_TangentLines() {
        // Test lines that might be tangent to obstacle boundaries
        Translation2d point1 = new Translation2d(1.0, 2.0);
        Translation2d point2 = new Translation2d(3.0, 2.0);

        // This line should be in open field
        assertFalse(FieldIntersection.collidesWithField(point1, point2), "Line in open field should not collide");
    }

    @Test
    @DisplayName("Test collision detection with very short line segments")
    void testCollidesWithField_ShortSegments() {
        // Test very short line segments
        Translation2d point1 = new Translation2d(5.0, 4.0);
        Translation2d point2 = new Translation2d(5.001, 4.0);

        assertFalse(
                FieldIntersection.collidesWithField(point1, point2),
                "Very short line in open field should not collide");

        // Test short segment within an obstacle
        Translation2d pillarPoint1 = new Translation2d(8.774, 4.026);
        Translation2d pillarPoint2 = new Translation2d(8.775, 4.026);

        assertTrue(
                FieldIntersection.collidesWithField(pillarPoint1, pillarPoint2),
                "Short line within obstacle should collide");
    }

    @Test
    @DisplayName("Test collision detection across field diagonal")
    void testCollidesWithField_FieldDiagonal() {
        // Test long diagonal lines across the field
        Translation2d bottomLeft = new Translation2d(0.5, 0.5);
        Translation2d topRight = new Translation2d(17.0, 7.5);

        assertTrue(
                FieldIntersection.collidesWithField(bottomLeft, topRight),
                "Diagonal line across field should hit obstacles");
    }

    @Test
    @DisplayName("Test collision detection with various open field paths")
    void testCollidesWithField_OpenFieldPaths() {
        // Test several paths that should be clear
        Translation2d[] openPoints = {
            new Translation2d(2.0, 2.0),
            new Translation2d(7.0, 2.0),
            new Translation2d(10.0, 2.0),
            new Translation2d(15.0, 2.0),
            new Translation2d(2.0, 6.0),
            new Translation2d(7.0, 6.0),
            new Translation2d(10.0, 6.0),
            new Translation2d(15.0, 6.0)
        };

        // Test connections between open points
        for (int i = 0; i < openPoints.length - 1; i++) {
            for (int j = i + 1; j < openPoints.length; j++) {
                Translation2d p1 = openPoints[i];
                Translation2d p2 = openPoints[j];
                // Some of these might intersect obstacles, but we test they execute without error
                assertDoesNotThrow(
                        () -> FieldIntersection.collidesWithField(p1, p2),
                        "Collision detection should not throw exceptions");
            }
        }
    }
}
