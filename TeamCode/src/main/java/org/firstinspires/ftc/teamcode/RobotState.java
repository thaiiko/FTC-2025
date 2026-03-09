package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public final class RobotState {
    private static final int MAX_ARTIFACTS = 3;
    private static final int MIN_ARTIFACT_INDEX = 0;
    private static final int MAX_ARTIFACT_INDEX = MAX_ARTIFACTS - 1;

    private static final int MIN_BALLS = 0;
    private static final int MAX_BALLS = 3;

    // Road Runner Pose2d heading is in radians.
    private static final double DEFAULT_HEADING_RADIANS = Math.PI;

    private static final Artifact[] artifacts = new Artifact[MAX_ARTIFACTS];
    private static int currentArtifactIndex;
    private static int ballsIn = MAX_BALLS;
    private static Pose2d currentPose = new Pose2d(0.0, 0.0, DEFAULT_HEADING_RADIANS);

    private RobotState() {
        throw new UnsupportedOperationException("Utility class");
    }

    public static Artifact[] getArtifacts() {
        return artifacts;
    }

    public static int getArtifactIndex() {
        return currentArtifactIndex;
    }

    public static Artifact[] setArtifacts(int index, Artifact artifact) {
        artifacts[clamp(index, MIN_ARTIFACT_INDEX, MAX_ARTIFACT_INDEX)] = artifact;
        return artifacts;
    }

    public static void setArtifactIndex(int index) {
        currentArtifactIndex = clamp(index, MIN_ARTIFACT_INDEX, MAX_ARTIFACT_INDEX);
    }

    public static void setBallsIn(int balls) {
        ballsIn = clamp(balls, MIN_BALLS, MAX_BALLS);
    }

    public static int getBallsIn() {
        return ballsIn;
    }

    public static void setCurrentPose(Pose2d pose) {
        if (pose != null) {
            currentPose = pose;
        }
    }

    public static Pose2d getCurrentPose() {
        return currentPose;
    }

    public static double getY(int balls) {
        int clampedBalls = clamp(balls, MIN_BALLS, MAX_BALLS);
        switch (clampedBalls) {
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

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }
}
