package org.firstinspires.ftc.teamcode.common;

/**
 * Enum for Limelight pipelines.
 */
public enum Pipeline {
    APRIL_TAG(0),
    BLUE_SPEC(1),
    RED_SPEC(2),
    YELLOW_SPEC(3);

    public final int id;

    Pipeline(int id) {
        this.id = id;
    }
}
