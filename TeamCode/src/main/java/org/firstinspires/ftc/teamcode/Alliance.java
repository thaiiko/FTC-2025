package org.firstinspires.ftc.teamcode;

public enum Alliance {
    RED(1),
    BLUE(-1);

    private final int value;

    Alliance(int value) {this.value = value;}
    public int getValue() {return value;}
}
