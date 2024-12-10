package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.stealthrobotics.library.Alliance;

public class PushBlocksPath extends PathManager {

    private static final Pose DEFAULT_START_POINT_BLUE = new Pose(8.25, 48, Math.toRadians(270));
    private static final Pose DEFAULT_START_POINT_RED = new Pose(144 - 8.25, 48, Math.toRadians(90));

    /**
     * Sample path using Bezier curves.
     *
     */
    public PushBlocksPath() {
        super(DEFAULT_START_POINT_BLUE);

        this.bluePathChain = buildBluePathChain();
        this.redPathChain = buildRedPathChain();

        if (Alliance.isRed()) {
            startPose = DEFAULT_START_POINT_RED;
            fullChain = redPathChain;
        } else {
            fullChain = bluePathChain;
        }

        // chop up the full path chain into separate path chains
        // so that other operations (like lifter movements) can be done at each segment
        createSegment(0, 1);
        createSegment(2, 3);
        buildSegments();
    }

    /**
     * the blue path chain
     * @return returns the path for when running as blue Alliance
     */
    protected PathChain buildBluePathChain() {
        PathBuilder builder = new PathBuilder();

        return builder
                .addPath(
                        // Move off wall
                        new BezierLine(
                                new Point(8.250, 48.000, Point.CARTESIAN),
                                new Point(12.000, 48.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        // Got past blocks
                        new BezierCurve(
                                new Point(12.000, 48.000, Point.CARTESIAN),
                                new Point(17.396, 35.154, Point.CARTESIAN),
                                new Point(63.000, 35.400, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // move down to blocks
                        new BezierLine(
                                new Point(63.000, 35.400, Point.CARTESIAN),
                                new Point(63.000, 25.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // go to observation station
                        new BezierLine(
                                new Point(63.000, 25.500, Point.CARTESIAN),
                                new Point(13.000, 25.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(6000) // 6 seconds
                .build();
    }

    /**
     * Builds the red path chain
     * @return Returns the Red PathChain for when on the Red Alliance
     */
    protected PathChain buildRedPathChain() {
        // Can just invert the blue chain to get the red chain
        return invertPathChain(buildBluePathChain());
    }
}
