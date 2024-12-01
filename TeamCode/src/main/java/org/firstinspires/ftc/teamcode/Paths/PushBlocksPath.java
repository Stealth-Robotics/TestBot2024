package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PushBlocksPath {
    public static final Pose StartPos = new Pose(8, 48, 0);
    public final PathChain pathChain;

    public PushBlocksPath() {
        pathChain = PushBlocks().build();
    }

    public static PathBuilder PushBlocks() {
        PathBuilder builder = new PathBuilder();

        return  builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.000, 48.000, Point.CARTESIAN),
                                new Point(10.286, 45.351, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(10.286, 45.351, Point.CARTESIAN),
                                new Point(22.442, 36.468, Point.CARTESIAN),
                                new Point(59.844, 36.701, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(59.844, 36.701, Point.CARTESIAN),
                                new Point(60.000, 26.577, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(60.000, 26.577, Point.CARTESIAN),
                                new Point(16.000, 27.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(16.000, 27.000, Point.CARTESIAN),
                                new Point(60.000, 23.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setReversed(true)
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(60.000, 23.500, Point.CARTESIAN),
                                new Point(60.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(60.000, 15.000, Point.CARTESIAN),
                                new Point(16.671, 14.859, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));
    }
}
