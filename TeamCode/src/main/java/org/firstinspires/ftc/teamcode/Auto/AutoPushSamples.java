package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Paths.PushSamplesPath;
import org.firstinspires.ftc.teamcode.common.StealthAutoMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * Example of pushing a blue block to the observation station.
 * if the word blue is used in name or group then Alliance is blue.
 * This can later be updated by the limelight if configured so
 */
public class AutoPushSamples extends StealthAutoMode {
    private PushSamplesPath path;

    @Override
    public void initialize() {
        super.initialize();
        path = new PushSamplesPath();
        commandGroup.addCommands(initPath(), runPath());
    }

    protected Command initPath() {
        return new InstantCommand(() ->
        {
            Follower follower = followerSubsystem.getFollower();
            if (avgPose != null) {
                path.setStartPose(avgPose);
            } else if (lastPose != null) {
                path.setStartPose(lastPose);
            }

            follower.setStartingPose(path.getStartPose());
            follower.update();
        });

    }

    private Command runPath() {
       assert (path.getSegmentCount() == 2);
        return
                new SequentialCommandGroup(
                        lss.startSetPositionCommand(0.5),
                        followerSubsystem.followPathCommand(path.getNextSegment()),
                        //lss.endSetPositionCommand(6000),
                        followerSubsystem.followPathCommand(path.getNextSegment(), true),
                        lss.setPositionCommand(0.01, 500)
                );
    }

    @Override
    public Command getAutoCommand() {
        return commandGroup;
    }
}
