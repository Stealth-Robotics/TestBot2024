package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.PushBlocksPath;
import org.firstinspires.ftc.teamcode.common.StealthAutoMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * Example of pushing a blue block to the observation station.
 * Right now it is setup for blue only will build a path reverser soon
 */
@Autonomous(name = "Blue Push Block", group = "examples")
public class AutoPushBlock extends StealthAutoMode {
    private PushBlocksPath path;

    @Override
    public void initialize() {
        super.initialize();
        path = new PushBlocksPath();
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
                        followerSubsystem.followPathCommand(path.getNextSegment()),
                        lss.setPositionCommand(.5),
                        followerSubsystem.followPathCommand(path.getNextSegment()),
                        lss.setPositionCommand(0.1)
                );
    }

    @Override
    public Command getAutoCommand() {
        return commandGroup;
    }
}
