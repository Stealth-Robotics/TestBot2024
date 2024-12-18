package org.firstinspires.ftc.teamcode.Paths;

import androidx.annotation.Nullable;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.stealthrobotics.library.Alliance;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages a sequence of path segments for robot navigation. Designed to be
 * extended by subclasses to provide specific path chains.
 * use <a href="https://pedro-path-generator.vercel.app/">...</a> to build your own paths
 */
public class PathManager {
    protected Pose startPose;

    protected PathChain bluePathChain;
    protected PathChain redPathChain;

    protected final List<PathChain> redPathSegments = new ArrayList<>();
    protected final List<PathChain> bluePathSegments = new ArrayList<>();

    protected final List<Pair<Integer, Integer>> pathSegmentIndices = new ArrayList<>();
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
        if (currentPath >= pathSegmentIndices.size() || pathSegmentIndices.isEmpty()) {
            return null;
        }

        return Alliance.isRed() ? redPathSegments.get(currentPath++) : bluePathSegments.get(currentPath++);
    }

    /**
     * Retrieves the full path chain for the current alliance.
     *
     * @return The full path chain for the current alliance.
     */
    public PathChain getFullPath() {
        return Alliance.isRed() ? redPathChain : bluePathChain;
    }

    /**
     * builds Red and Blue segments to break the full paths up into smaller segments
     * This allows doing other actions at the end of each segment and not having to
     * break up the entire path you get from <a href="https://pedro-path-generator.vercel.app/">...</a>
     */
    protected void buildSegments()
    {
        bluePathSegments.clear();
        redPathSegments.clear();
        currentPath = 0;
        
        if (pathSegmentIndices.isEmpty())
        {
            bluePathSegments.add(bluePathChain);
            redPathSegments.add(redPathChain);
            return;
        }
        
        for (int i = 0; i < pathSegmentIndices.size(); i++) {
            int start = pathSegmentIndices.get(i).fst;
            int end = pathSegmentIndices.get(i).snd;
            bluePathSegments.add(buildSegment(bluePathChain, start, end));
            redPathSegments.add(buildSegment(redPathChain, start, end));
        }
    }

    /**
     * Builds a path segment from the given path chain.
     * @param pathChain The path chain to build the segment from.
     * @param start start index of the path chain.
     * @param end end index of the path chain.
     * @return The built path segment.
     */
    private PathChain buildSegment(PathChain pathChain, int start, int end)
    {
       ArrayList<Path> paths = new ArrayList<>();
       ArrayList<PathCallback> callbacks = new ArrayList<>();

        for (int i = start,  k = 0; i <= end; i++) {

            if (i >= pathChain.size()) {
                break;
            }

            paths.add(pathChain.getPath(i));
            PathCallback cb = pathChain.getCallback(i);
            if (cb != null) {
                callbacks.add(cb.clone(k++));
            }
        }

        PathChain pc = new PathChain(paths);
        pc.setCallbacks(callbacks);
        return pc;
    }

    /**
     * Retrieves the total number of path segments.
     * @return The number of path segments.
     */
    public int getSegmentCount() {
        return pathSegmentIndices.size();
    }

    /**
     * Sets the start pose for the path manager.
     *
     * @param startPose The new start pose.
     */
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;

        // Set alliance based on field pose
        if (startPose.getX() > (FollowerConstants.FIELD_SIZE_X_INCHES / 2)) {
            Alliance.set(Alliance.RED);
          } else {
            Alliance.set(Alliance.BLUE);
        }
    }

    /**
     * Retrieves the start pose for the path manager.
     *
     * @return The start pose.
     */
    public Pose getStartPose() {
        return this.startPose;
    }

    protected void createSegment(int start, int end) {
        pathSegmentIndices.add(new Pair<>(start, end));

    }

    protected void createSegment(int index) {
        pathSegmentIndices.add(new Pair<>(index, index));
    }

    public static PathChain invertPathChain(PathChain pathChain) {
        ArrayList<Path> invertedPaths = new ArrayList<>();
        for (int i =0; i < pathChain.size(); i++) {
            Path path = pathChain.getPath(i);
            invertedPaths.add(path.getInvertedCopy());
        }

        PathChain invertedPathChain = new PathChain(invertedPaths);
        invertedPathChain.setCallbacks(pathChain.getCallbacks());
        return invertedPathChain;
    }
}
