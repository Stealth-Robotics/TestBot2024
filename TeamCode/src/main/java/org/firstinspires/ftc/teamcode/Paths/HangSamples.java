package org.firstinspires.ftc.teamcode.Paths;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.stealthrobotics.library.Alliance;


public class HangSamples extends PathManager {
    // Sets the default start point for this path. Use Limelight during whileWaitingToStart to
    // update this location in your Command code.
    private static final Pose DEFAULT_START_POINT_BLUE = new Pose(
            9,
            63,
            Math.toRadians(0));

    private static final Pose DEFAULT_START_POINT_RED = new Pose(
             // Use 180 if using limelight to get field position   
            // FollowerConstants.FIELD_SIZE_X_INCHES - 9,
            // FollowerConstants.FIELD_SIZE_Y_INCHES - 63,
            // Math.toRadians(180));
            9,
            63,
            Math.toRadians(0));

    public HangSamples() {
        super(DEFAULT_START_POINT_BLUE);

        if (Alliance.isRed()) {
            startPose = DEFAULT_START_POINT_RED;
        }

        this.bluePathChain = buildBluePathChain();
        this.redPathChain = buildRedPathChain();
        // chop up the full path chain into separate path chains
        // so that other operations (like lifter movements) can be done at each segment
        // createSegment(0);
        // createSegment(1);
        buildSegments();
    }

    public PathChain buildBluePathChain() {
        PathBuilder builder = new PathBuilder();

        return builder
        .addPath(
            // Line 1
            new BezierLine(
              new Point(9.000, 63.000, Point.CARTESIAN),
              new Point(39.000, 60.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(0))
          .addPath(
            // Line 2
            new BezierCurve(
              new Point(39.000, 60.000, Point.CARTESIAN),
              new Point(31.800, 65.000, Point.CARTESIAN),
              new Point(8.000, 35.000, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
          .addPath(
            // Line 3
            new BezierCurve(
              new Point(8.000, 35.000, Point.CARTESIAN),
              new Point(30.900, 66.300, Point.CARTESIAN),
              new Point(39.000, 60.500, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
          .addPath(
            // Line 4
            new BezierCurve(
              new Point(39.000, 60.500, Point.CARTESIAN),
              new Point(28.800, 63.000, Point.CARTESIAN),
              new Point(29.200, 32.400, Point.CARTESIAN),
              new Point(60.000, 36.000, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
          .addPath(
            // Line 5
            new BezierLine(
              new Point(60.000, 36.000, Point.CARTESIAN),
              new Point(60.000, 8.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 6
            new BezierLine(
              new Point(60.000, 8.000, Point.CARTESIAN),
              new Point(18.000, 8.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 7
            new BezierLine(
              new Point(18.000, 8.000, Point.CARTESIAN),
              new Point(60.000, 8.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 8
            new BezierLine(
              new Point(60.000, 8.000, Point.CARTESIAN),
              new Point(60.000, 14.500, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 9
            new BezierLine(
              new Point(60.000, 14.500, Point.CARTESIAN),
              new Point(18.000, 14.500, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 10
            new BezierLine(
              new Point(18.000, 14.500, Point.CARTESIAN),
              new Point(60.000, 14.500, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 11
            new BezierLine(
              new Point(60.000, 14.500, Point.CARTESIAN),
              new Point(60.000, 24.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 12
            new BezierLine(
              new Point(60.000, 24.000, Point.CARTESIAN),
              new Point(8.000, 24.000, Point.CARTESIAN)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(180))
          .addPath(
            // Line 13
            new BezierCurve(
              new Point(8.000, 24.000, Point.CARTESIAN),
              new Point(28.600, 64.500, Point.CARTESIAN),
              new Point(39.000, 60.700, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
          .addPath(
            // Line 14
            new BezierCurve(
              new Point(39.000, 60.700, Point.CARTESIAN),
              new Point(30.600, 64.500, Point.CARTESIAN),
              new Point(8.000, 35.000, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
          .addPath(
            // Line 15
            new BezierCurve(
              new Point(8.000, 35.000, Point.CARTESIAN),
              new Point(33.600, 65.000, Point.CARTESIAN),
              new Point(39.000, 61.000, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
          .addPath(
            // Line 16
            new BezierCurve(
              new Point(39.000, 61.000, Point.CARTESIAN),
              new Point(27.400, 64.100, Point.CARTESIAN),
              new Point(8.000, 35.000, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
          .addPath(
            // Line 17
            new BezierCurve(
              new Point(8.000, 35.000, Point.CARTESIAN),
              new Point(26.700, 63.500, Point.CARTESIAN),
              new Point(39.000, 61.500, Point.CARTESIAN)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
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
