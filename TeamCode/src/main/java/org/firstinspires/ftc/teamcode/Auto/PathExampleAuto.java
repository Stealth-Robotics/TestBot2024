package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.OutAndBackPath;
import org.firstinspires.ftc.teamcode.common.AutoBase;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "Blue pathing example", group = "examples")
public class PathExampleAuto extends AutoBase {


   private OutAndBackPath path;

   private final SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {

        this.Init();
        telemetryA.addLine("Example path of using the follower");
        path = new OutAndBackPath(this);
        commandGroup.addCommands(initPath(), DoPath());
    }

    @Override
    public Command getAutoCommand() {
        return commandGroup;
    }

    private Command initPath() {
        return new InstantCommand(()->
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

    private Command DoPath() {
        Follower follower = followerSubsystem.getFollower();
        assert (path.pathChain.size() == 2);

        return new SequentialCommandGroup(
                new InstantCommand(() ->follower.followPath(path.pathChain.getPath(0))),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                        getExtender().setPosition(.99);
                        getLifter().setPosition(.99);
                }),
                new WaitCommand(500),
                new InstantCommand(() ->follower.followPath(path.pathChain.getPath(1))),
                new WaitCommand(1000),
                new InstantCommand(() ->
                {
                    getExtender().setPosition(0.01);
                    getLifter().setPosition(0.01);
                })

        );
    }

}
