package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import javax.annotation.Nullable;

/**
 * This is the PathChain class. This class handles chaining together multiple Paths into a larger
 * collection of Paths that can be run continuously. Additionally, this allows for the addition of
 * PathCallbacks to specific Paths in the PathChain, allowing for non-blocking code to be run in
 * the middle of a PathChain.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class PathChain {
    private ArrayList<Path> pathChain = new ArrayList<>();

    private final Map<Integer, PathCallback> callbackMap = new HashMap<>();

    /**
     * This creates a new PathChain from some specified Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     */
    public PathChain(Path... paths) {
        pathChain.addAll(Arrays.asList(paths));
    }

    /**
     * This creates a new PathChain from an ArrayList of Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of Paths.
     */
    public PathChain(ArrayList<Path> paths) {
        pathChain = paths;
    }

    /**
     * This returns the Path on the PathChain at a specified index.
     *
     * @param index the index.
     * @return returns the Path at the index.
     */
    public Path getPath(int index) {
        return pathChain.get(index);
    }

    /**
     * This returns the size of the PathChain.
     *
     * @return returns the size of the PathChain.
     */
    public int size() {
        return pathChain.size();
    }

    /**
     * This sets the PathCallbacks of the PathChain with some specified PathCallbacks.
     *
     * @param callbacks the specified PathCallbacks.
     */
    public void setCallbacks(PathCallback... callbacks) {
        for (PathCallback callback : callbacks) {
            this.callbackMap.put(callback.getIndex(), callback);
        }
    }

    /**
     * This sets the PathCallbacks of the PathChain with an ArrayList of PathCallbacks.
     *
     * @param callbacks the ArrayList of PathCallbacks.
     */
    public void setCallbacks(ArrayList<PathCallback> callbacks) {

        for (PathCallback callback : callbacks) {
            this.callbackMap.put(callback.getIndex(), callback);
        }
    }

    /**
     * This sets the PathCallbacks of the PathChain with a Map of PathCallbacks.
     *
     * @param callbacks the Map of PathCallbacks.
     */
    public void setCallbacks(Collection<PathCallback> callbacks) {
        for (PathCallback callback : callbacks) {
            this.callbackMap.put(callback.getIndex(), callback);
        }
    }

    /**
     * This sets the PathCallbacks of the PathChain with a Map of PathCallbacks.
     *
     * @param callbacks the Map of PathCallbacks.
     */
    public void setCallbacks(Map<Integer, PathCallback> callbacks) {
        this.callbackMap.putAll(callbacks);
    }

    /**
     * This returns the PathCallbacks of this PathChain in an ArrayList.
     *
     * @return returns the PathCallbacks.
     */
    public Collection<PathCallback> getCallbacks() {
        return callbackMap.values();
    }

    /**
     * This returns the PathCallback at a specified index.
     *
     * @param index the index.
     * @return returns the PathCallback at the index.
     */
    @Nullable
    public PathCallback getCallback(int index) {
        return callbackMap.get(index);
    }
}
