package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.common.AutoBase;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class OutAndBackPath {

    private Pose startPose = new Pose(9, 72, Math.toRadians(0));
    public final PathChain pathChain;
    private final AutoBase autoBase;
    public OutAndBackPath(AutoBase autoBase) {
       this.autoBase = autoBase;
        pathChain = getPathChain();
    }

    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    public Pose getStartPose() {
        return this.startPose;
    }

    public PathChain getPathChain() {
        PathBuilder builder = new PathBuilder();
        LifterSubsystem lss = autoBase.getLifter();

        return builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 72.000, Point.CARTESIAN),
                                new Point(48.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.addParametricCallback(.5, ()-> lss.setPosition(.5))

                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(48.000, 72.000, Point.CARTESIAN),
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

    }
}
