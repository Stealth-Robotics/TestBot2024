package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

    /**
     * Called aft er the Init button is pushed add any specific
     * code that sets up the robot here
     */
    @Override
    public void initialize() {
        super.initialize();
        path = new PushSamplesPath();
        commandGroup.addCommands(initPath(), runPath());
    }

    /**
     * Called by the op mode to get the list of command to run during play
     * @return Command(s) to run
     */
    @Override
    public Command getAutoCommand() {
        return commandGroup;
    }

    /**
     * Put code here that you want to do first right after pressing play
     * @return a Command to be run
     */
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

    /**
     * This is were you can put the majority of your code to more the bot
     * and operate different components. The sequential command group will
     * get be called by the scheduler one at a time until all are done.
     * @return a Command to be run
     */
    private Command runPath() {
       assert (path.getSegmentCount() == 2);
        return
                new SequentialCommandGroup(
                        followerSubsystem.followPathCommand(path.getNextSegment()),
                        lss.startSetPositionCommand(0.5), // sets moving the arm but does not wait
                        followerSubsystem.followPathCommand(path.getNextSegment(), true),
                        doArmMovement()
                );
    }

    /**
     * Example of combining multiple movements into a single command group
     * If you find you need to do the same command multiple times a good idea is to do it this way
     * @return Command to be run
     */
    private Command doArmMovement() {
        return new SequentialCommandGroup(
                lss.setPositionCommand(0.01, 100),
                ess.setPositionCommand(0.5, 500),
                new WaitCommand(1000),
                ess.setPositionCommand(0.01, 500)
        );
    }

}
