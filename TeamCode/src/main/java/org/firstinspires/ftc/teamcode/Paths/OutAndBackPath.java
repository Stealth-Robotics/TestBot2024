package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.stealthrobotics.library.Alliance;

/**
 * Example of building a path chain. and chunking it into segments.
 * This allows for doing other operations at the end of each segment like get a sample etc.
 */
public class OutAndBackPath extends PathManager {

    // Sets the default start point for this path. Use Limelight during whileWaitingToStart to
    // update this location in your Command code.
    private static final Pose DEFAULT_START_POINT_BLUE = new Pose(
            9, 72,
            Math.toRadians(0));

    private static final Pose DEFAULT_START_POINT_RED = new Pose(
            FollowerConstants.FIELD_SIZE_X_INCHES - 9,
            FollowerConstants.FIELD_SIZE_Y_INCHES - 72,
            Math.toRadians(180));
    public OutAndBackPath() {
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
        buildSegments();
    }

    public PathChain buildBluePathChain() {
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

    /**
     * Builds the red path chain
     * @return Returns the Red PathChain for when on the Red Alliance
     */
    protected PathChain buildRedPathChain() {
        // Can just invert the blue chain to get the red chain
        return invertPathChain(buildBluePathChain());
    }
}
