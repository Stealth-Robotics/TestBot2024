package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class OutAndBackPath {

    public Pose StartPose = new Pose(8.000, 48.000, Math.toRadians(270));
    public final PathChain pathChain;
    private final LifterSubsystem lss;
    public OutAndBackPath(LifterSubsystem lss) {
        this.lss = lss;
        pathChain = getPathChain();
    }

    public PathChain getPathChain() {
        PathBuilder builder = new PathBuilder();

        return builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(StartPose.getX(), StartPose.getY(), Point.CARTESIAN),
                                new Point(14.000, 48.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierCurve(
                                new Point(14.000, 48.000,Point.CARTESIAN),
                                new Point(32.859, 29.700, Point.CARTESIAN),
                                new Point(60.000, 34.800, Point.CARTESIAN)
                        )
                )
                //.addParametricCallback(.5 , ()-> lss.doPosition(.5))
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Point(60.000, 34.800, Point.CARTESIAN),
                                new Point(24, 48, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(StartPose.getHeading())
                //.addTemporalCallback(0, ()-> lss.doPosition(.001))
                .build();

    }
}
