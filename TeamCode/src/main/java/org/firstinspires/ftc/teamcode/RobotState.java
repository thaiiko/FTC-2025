package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotState {
    static int ballsIn = 3;
    static Pose2d currentPose = new Pose2d(0, 0, 180);

    public static void setBallsIn(int balls) {
        if (balls < 0) {
            balls = 0;
        } else if (balls > 3) {
            balls = 3;
        }
        ballsIn = balls;
    }
    public static void setCurrentPose(Pose2d pose) {
        currentPose = pose;
    }
    public static Pose2d getCurrentPose() {
        return currentPose;
    }
    public static int getBallsIn() {
        return ballsIn;
    }
    public static double getY(int balls) {
        setBallsIn(balls);
        switch (ballsIn) {
            case 0:
                return 58;
            case 1:
                return 52;
            case 2:
                return 48;
            case 3:
                return 31;
            default:
                return 58;
        }
    }
}
