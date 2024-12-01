package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PathSquare {

    public static final Pose StartPose = new Pose(38.000, 100, 0);

    public static PathChain getPathChain() {

        PathBuilder builder = new PathBuilder();

        return builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(38.000, 100.000, Point.CARTESIAN),
                                new Point(100.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(100.000, 100.000, Point.CARTESIAN),
                                new Point(100.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(38.000, 36.000, Point.CARTESIAN),
                                new Point(38.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(38.000, 36.000, Point.CARTESIAN),
                                new Point(38.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(false).build();
    }
}
