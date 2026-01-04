package org.firstinspires.ftc.teamcode;

public class RobotState {
    static int ballsIn = 3;

    public static void setBallsIn(int balls) {
        if (balls < 0) {
            balls = 0;
        } else if (balls > 3) {
            balls = 3;
        }
        ballsIn = balls;
    }
    public static int getBallsIn() {
        return ballsIn;
    }
    public static double getY(int balls) {
        setBallsIn(3);
        switch (balls) {
            case 0:
                return -37;
            case 1:
                return -32;
            case 2:
                return -27;
            case 3:
                return -24;
            default:
                return -38;
        }
    }
}
