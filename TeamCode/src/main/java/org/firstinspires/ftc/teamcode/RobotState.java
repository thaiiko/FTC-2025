package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.jetbrains.annotations.NotNull;

public class RobotState {
    public static NormalizedRGBA[] colors = null;
    public static Ball[] balls = null;
    public static boolean ballSeen;

    public static void updateBalls(@NotNull RobotHardware robot) {
        colors[0] = robot.color1.getNormalizedColors();
        colors[1] = robot.color2.getNormalizedColors();

        NormalizedRGBA color1 = robot.color1.getNormalizedColors();

        if (color1.red  * 10 > 0.5 && color1.blue * 10 > 0.5) {
            balls[ballsIn] = Ball.PURPLE;
            ballSeen = true;
        } else if (color1.green > color1.blue && color1.green > color1.red && color1.red * 10< 0.5 && !ballSeen) {
            balls[ballsIn] = Ball.PURPLE;
            ballSeen = true;
        } else if (ballSeen) {
            ballSeen = false;
        }


//        for (int i = 0; i < colors.length - 1; i++) {
//            if (colors[i].red > 0.5 && colors[i].blue > 0.5) {
//                balls[i] = Ball.PURPLE;
//                ballSeen = true;
//            } else if (colors[i].green > 0.6) {
//                balls[i] = Ball.GREEN;
//                ballSeen = true;
//            } else {
//                ballSeen = false;
//            }
//        }

    }

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
