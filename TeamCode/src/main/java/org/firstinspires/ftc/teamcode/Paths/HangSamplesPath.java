package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.stealthrobotics.library.Alliance;


public class HangSamplesPath extends PathManager {

    private static final double rotationEnd = 0.9;
    private static final double pickupAngle = Math.toRadians(179.99);
    private static final double scoreAngle = Math.toRadians(0);

    private static final double pickupX = 11;
    private static final double pickupY = 35;
    private static final double hangX = 34.5;
    
    // Sets the default start point for this path. Use Limelight during whileWaitingToStart to
    // update this location in your Command code.
    private static final Pose DEFAULT_START_POINT_BLUE = new Pose(
            8.5,
            63,
            Math.toRadians(0));

    private static final Pose DEFAULT_START_POINT_RED = new Pose(
             // Use 180 if using limelight to get field position   
            // FollowerConstants.FIELD_SIZE_X_INCHES - 9,
            // FollowerConstants.FIELD_SIZE_Y_INCHES - 63,
            // Math.toRadians(180));
            8.5,
            63,
            Math.toRadians(0));

    public HangSamplesPath() {
        super(DEFAULT_START_POINT_BLUE);

        if (Alliance.isRed()) {
            startPose = DEFAULT_START_POINT_RED;
        }

        this.bluePathChain = buildBluePathChain();
        this.redPathChain = buildRedPathChain();

        // chop up the full path chain into separate path chains
        // so that other operations (like lifter movements) can be done at each segment
        createSegment(0);
        createSegment(1);
        createSegment(2);
        createSegment(3, 11);
        createSegment(12);
        createSegment(13);
        createSegment(14);
        createSegment(15);
        createSegment(16);
        buildSegments();
    }

    protected PathChain buildBluePathChain() {
        PathBuilder builder = new PathBuilder();

        return builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.500, 63.000, Point.CARTESIAN),
                                new Point(hangX, 61.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(hangX, 61.000, Point.CARTESIAN),
                                new Point(20.600, 65.900, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(scoreAngle, pickupAngle, rotationEnd)
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(20.300, 66.600, Point.CARTESIAN),
                                new Point(hangX, 61.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(pickupAngle, scoreAngle, rotationEnd)
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(hangX, 61.500, Point.CARTESIAN),
                                new Point(20.600, 59.400, Point.CARTESIAN),
                                new Point(26.600, 38, Point.CARTESIAN),
                                new Point(60.000, 36, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(scoreAngle, pickupAngle, rotationEnd)
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(60.000, 36, Point.CARTESIAN),
                                new Point(60.000, 28.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(60.000, 30.000, Point.CARTESIAN),
                                new Point(16.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(16.000, 30, Point.CARTESIAN),
                                new Point(60.000, 29, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(60.000, 29, Point.CARTESIAN),
                                new Point(60.000, 25, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(60.000, 25, Point.CARTESIAN),
                                new Point(16.000, 25, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(16.000, 25, Point.CARTESIAN),
                                new Point(60.000, 25, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(60.000, 25, Point.CARTESIAN),
                                new Point(60.000, 18, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(60.000, 18, Point.CARTESIAN),
                                new Point(16.000, 18, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(16.000, 18, Point.CARTESIAN),
                                new Point(17.100, 36.000, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(20.300, 63.900, Point.CARTESIAN),
                                new Point(hangX, 62.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), scoreAngle, rotationEnd)
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(hangX, 62.000, Point.CARTESIAN),
                                new Point(19.600, 61.100, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), pickupAngle, rotationEnd)
                .addPath(
                        // Line 16
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(29.500, 66.400, Point.CARTESIAN),
                                new Point(hangX, 62.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(pickupAngle, scoreAngle, rotationEnd)
                .addPath(
                        // Line 17
                        new BezierCurve(
                                new Point(hangX, 62.500, Point.CARTESIAN),
                                new Point(27.300, 64.800, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(scoreAngle, pickupAngle, rotationEnd)
                .addPath(
                        // Line 18
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(27.818, 66.390, Point.CARTESIAN),
                                new Point(hangX, 63.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(pickupAngle, scoreAngle, rotationEnd)
                .addPath(
                        // Line 19
                        new BezierCurve(
                                new Point(hangX, 63.000, Point.CARTESIAN),
                                new Point(29.221, 62.649, Point.CARTESIAN),
                                new Point(12.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();
    }

    /**
     * Builds the red path chain
     * @return Returns the Red PathChain for when on the Red Alliance
     */
    protected PathChain buildRedPathChain() {
        // Invert if using the Limelight to get position
        //return invertPathChain(buildBluePathChain());
        return bluePathChain;
    }
}
