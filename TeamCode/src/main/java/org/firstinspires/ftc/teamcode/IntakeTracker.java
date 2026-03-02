package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.jetbrains.annotations.NotNull;

public class IntakeTracker {
    private final RobotHardware robot;
    private boolean lastBallSeenIntake;
    private boolean lastBallSeenShoot;

    public IntakeTracker(RobotHardware robot) {
        this.robot = robot;
    }

    public void trackIntake() {
        double hue1 = JavaUtil.colorToHue(robot.color1.getNormalizedColors().toColor());
        double hue2 = JavaUtil.colorToHue(robot.color2.getNormalizedColors().toColor());

        Artifact detectedArtifact = detectArtifact(hue1, hue2);
        boolean ballCurrentlySeen = detectedArtifact != null;

        if (ballCurrentlySeen && !lastBallSeenIntake) {
            int currentIndex = RobotState.getArtifactIndex();
            if (currentIndex < Constants.Intake.MAX_BALLS) {
                RobotState.setArtifacts(currentIndex, detectedArtifact);
                updateIndicatorLight(currentIndex, detectedArtifact);
                RobotState.setArtifactIndex(currentIndex + 1);
                RobotState.setBallsIn(RobotState.getBallsIn() + 1);
            }
        }

        lastBallSeenIntake = ballCurrentlySeen;
    }

    public void trackShoot(boolean shooterOn) {
        if (!shooterOn) {
            lastBallSeenShoot = false;
            return;
        }

        boolean ballCurrentlySeen = robot.distance1.getState();

        if (!ballCurrentlySeen && lastBallSeenShoot) {
            int ballsIn = RobotState.getBallsIn();
            if (ballsIn > 0) {
                shiftArtifactsDown();
                RobotState.setBallsIn(ballsIn - 1);
                RobotState.setArtifactIndex(Math.max(0, RobotState.getArtifactIndex() - 1));
                updateAllIndicatorLights();
            }
        }

        lastBallSeenShoot = ballCurrentlySeen;
    }

    public Action trackIntakeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                trackIntake();
                telemetryPacket.put("balls", RobotState.getBallsIn());
                telemetryPacket.put("artifactIndex", RobotState.getArtifactIndex());
                return RobotState.getBallsIn() < Constants.Intake.MAX_BALLS;
            }
        };
    }

    private void shiftArtifactsDown() {
        Artifact[] artifacts = RobotState.getArtifacts();
        for (int i = 0; i < artifacts.length - 1; i++) {
            RobotState.setArtifacts(i, artifacts[i + 1]);
        }
        RobotState.setArtifacts(artifacts.length - 1, null);
    }

    private void updateAllIndicatorLights() {
        Artifact[] artifacts = RobotState.getArtifacts();
        for (int i = 0; i < Constants.Intake.MAX_BALLS; i++) {
            if (artifacts[i] != null) {
                updateIndicatorLight(i, artifacts[i]);
            } else {
                clearIndicatorLight(i);
            }
        }
    }

    private Artifact detectArtifact(double hue1, double hue2) {
        if (isInRange(hue1, Constants.Intake.GREEN_HUE_MIN, Constants.Intake.GREEN_HUE_MAX)
                || isInRange(hue2, Constants.Intake.GREEN_HUE_MIN, Constants.Intake.GREEN_HUE_MAX)) {
            return Artifact.GREEN;
        }

        if (isInRange(hue1, Constants.Intake.PURPLE_HUE_MIN, Constants.Intake.PURPLE_HUE_MAX)
                || isInRange(hue2, Constants.Intake.PURPLE_HUE_MIN, Constants.Intake.PURPLE_HUE_MAX)) {
            return Artifact.PURPLE;
        }

        return null;
    }

    private boolean isInRange(double value, double min, double max) {
        return value > min && value < max;
    }

    private void updateIndicatorLight(int slot, Artifact artifact) {
        double position = artifact == Artifact.PURPLE
                ? Constants.Intake.LIGHT_PURPLE
                : Constants.Intake.LIGHT_GREEN;

        switch (slot) {
            case 0:
                robot.light2.setPosition(position);
                break;
            case 1:
                robot.light3.setPosition(position);
                break;
            case 2:
                robot.light4.setPosition(position);
                break;
            default:
                break;
        }
    }

    private void clearIndicatorLight(int slot) {
        switch (slot) {
            case 0:
                robot.light2.setPosition(0.0);
                break;
            case 1:
                robot.light3.setPosition(0.0);
                break;
            case 2:
                robot.light4.setPosition(0.0);
                break;
            default:
                break;
        }
    }
}
