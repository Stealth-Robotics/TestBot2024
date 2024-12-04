package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PushBlocksPath extends PathManager {

    private static final Pose DEFAULT_START_POINT = new Pose(8.25, 48, Math.toRadians(270));

    /**
     * Sample path using Bezier curves.
     *
     */
    public PushBlocksPath() {
        super(DEFAULT_START_POINT);
        fullChain = buildFullPathChain();
        // chop up the full path chain into separate path chains
        // so that other operations (like lifter movements) can be done at each segment
        addPathSegment(buildSegment(0, 1));
        addPathSegment(buildSegment(2, 3));
    }

    public PathChain buildFullPathChain() {
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
                .build();
    }
}
