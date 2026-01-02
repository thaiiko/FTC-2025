package org.firstinspires.ftc.teamcode;

public enum Pipeline {
    MOTIF_PIPELINE(0),
    BLUE_PIPELINE(1),
    RED_PIPELINE(2),
    BLUE_GOAL(20),
    GPP(21),
    PGP(22),
    PPG(23),
    RED_GOAL(24);

    private final int value;

    Pipeline(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
