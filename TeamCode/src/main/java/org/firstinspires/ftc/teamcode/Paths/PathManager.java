package org.firstinspires.ftc.teamcode.Paths;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages a sequence of path segments for robot navigation. Designed to be
 * extended by subclasses to provide specific path chains.
 */
public class PathManager {
    protected Pose startPose;
    protected PathChain fullChain;

    protected final List<PathChain> pathSegments = new ArrayList<>();
    protected int currentPath = 0;

    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param startPose Default start location can be updated later
     *                  using setStartPose() during whileWaitingToStart
     */
    public PathManager(Pose startPose) {
        this.startPose = startPose;
    }

    /**
     * Retrieves the next path segment in the sequence.
     *
     * @return The next path segment, or null if there are no more segments.
     */
    @Nullable
    public PathChain getNextSegment() {
        if (currentPath >= pathSegments.size() || pathSegments.isEmpty()) {
            return null;
        }

        return pathSegments.get(currentPath++);
    }

    public int getSegmentCount() {
        return pathSegments.size();
    }

    /**
     * Adds a path segment to the sequence.
     *
     * @param pathSegment The path segment to add.
     */
    public void addPathSegment(PathChain pathSegment) {
        pathSegments.add(pathSegment);
    }

    /**
     * Sets the start pose for the path manager.
     *
     * @param startPose The new start pose.
     */
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    /**
     * Retrieves the start pose for the path manager.
     *
     * @return The start pose.
     */
    public Pose getStartPose() {
        return this.startPose;
    }

    /**
     * Returns a specific path chain from the given index
     * @param index The index of the path chain to retrieve
     * @return The path chain at the given index
     */
    protected PathChain buildSegment(int index) {

        return this.buildSegment(index, index);
    }

    /**
     * Builds a path chain from the given start and end indices
     * @param start The starting index of the path chain
     * @param end The ending index of the path chain
     * @return The built path chain
     */
    protected PathChain buildSegment(int start, int end) {
        PathBuilder builder = new PathBuilder();

        for (int i = start; i <= end; i++) {
            builder.addPath(fullChain.getPath(i));
        }

        return builder.build();
    }

}
