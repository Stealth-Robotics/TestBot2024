package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * Example of building a path chain. and chunking it into segments.
 * This allows for doing other operations at the end of each segment like get a sample etc.
 */
public class OutAndBackPath extends PathManager {

    // Sets the default start point for this path. Use Limelight during whileWaitingToStart to
    // update this location in your Command code.
    private static final Pose DEFAULT_START_POINT = new Pose(9, 72, Math.toRadians(0));
    
    public OutAndBackPath() {
        super(DEFAULT_START_POINT);
        fullChain = buildFullPathChain();
        // chop up the full path chain into separate path chains
        // so that other operations (like lifter movements) can be done at each segment
        addPathSegment(buildSegment(0));
        addPathSegment(buildSegment(1));
    }
   

    public PathChain buildFullPathChain() {
        PathBuilder builder = new PathBuilder();

        return builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                // You can use hard coded start points so the robot first 
                                // goes to that location or you can do this to go from where ever 
                                // the robot is currently at. Down side is that you might run into stuff
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(48.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.addParametricCallback(.5, ()-> lss.setPosition(.5))

                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(48.000, 24.000, Point.CARTESIAN),
                                new Point(9.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

    }
}
