package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

public class RobotHardware extends MecanumDrive {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx lShooter;
    public DcMotorEx rShooter;
    public DcMotorEx index;
    public DcMotorEx intake;

    public IMU imu;
    public GoBildaPrismDriver prism;
    public Limelight3A limelight;
    public Servo light1;
    public Servo light2;
    public Servo light3;
    public Servo light4;

    public DigitalChannel distance1;
    public RevColorSensorV3 color1;
    public RevColorSensorV3 color2;

    public HashMap<Double, Double> speedMap = new HashMap<>();
//    public DistanceSensor distanceSensor;
    public RobotHardware(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");



        distance1 = hardwareMap.get(DigitalChannel.class, "distance1");
        distance1.setMode(DigitalChannel.Mode.INPUT);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");
        light4 = hardwareMap.get(Servo.class, "light4");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");
        index = hardwareMap.get(DcMotorEx.class, "index");
        intake = hardwareMap.get(DcMotorEx.class, "intake");


        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        lShooter.setDirection(DcMotorSimple.Direction.FORWARD);
//        rShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,  new PIDFCoefficients(24, 0.1, 0, 26));
        rShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,  new PIDFCoefficients(24, 0.1, 0, 26));

        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.setPollRateHz(100);
        limelight.start();
    }

    /**
     * Sets the power level for both shooter motors.
     *
     * @param power The power level to set, ranging from -1.0 to 1.0.
     */
    public void setShooterPower(double power) {
        lShooter.setPower(power);
        rShooter.setPower(power);
    }

    /**
     * Gets the current velocity of the left shooter motor.
     *
     * @return The velocity in encoder ticks per second.
     */
    public double getVelocity() {
        return lShooter.getVelocity();
    }

    /**
     * Sets the target velocity for both shooter motors using the built-in PIDF controller.
     *
     * @param velocity The target velocity in encoder ticks per second.
     */
    public void setShooterVelocity(double velocity) {
        lShooter.setVelocity(velocity);
        rShooter.setVelocity(velocity);
    }

    /**
     * Creates an action that starts the intake motor.
     * <p>
     * This is a one-shot action that sets the intake motor power to 0.8
     * and immediately completes.
     * </p>
     *
     * @return An Action that starts the intake and returns {@code false} immediately.
     */
    public Action startIntake() {
        return new IntakeStart();
    }

    public class IntakeStart implements Action {
        private boolean initalized = false;

        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            if (!initalized) {
                intake.setPower(0.8);
                initalized = true;
            }

            return false;
        }
    }

    /**
     * Creates an action that tracks ball intake using color sensors.
     * <p>
     * This action continuously monitors the color sensors to detect incoming balls,
     * classifies them as GREEN or PURPLE based on hue values, and updates the
     * corresponding indicator lights.
     * </p>
     *
     * @return An Action that returns {@code true} while fewer than 3 balls have been
     *         collected, and {@code false} once 3 or more balls are detected
     *         (signaling the action is complete).
     */
    public Action trackIntake() {
        return new Action() {
            // Hue ranges for ball color detection
            private static final double GREEN_HUE_MIN = 48;
            private static final double GREEN_HUE_MAX = 165;
            private static final double PURPLE_HUE_MIN = 211;
            private static final double PURPLE_HUE_MAX = 338;

            // Light positions for colors
            private static final double LIGHT_PURPLE = 0.7;
            private static final double LIGHT_GREEN = 0.5;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double hue1 = JavaUtil.colorToHue(color1.getNormalizedColors().toColor());
                double hue2 = JavaUtil.colorToHue(color2.getNormalizedColors().toColor());

                Artifact detectedArtifact = detectArtifact(hue1, hue2);
                if (detectedArtifact != null) {
                    RobotState.setArtifacts(RobotState.getArtifactIndex(), detectedArtifact);
                    updateIndicatorLight(RobotState.getArtifactIndex(), detectedArtifact);
                } else {
                    RobotState.setArtifactIndex(RobotState.getArtifactIndex() + 1);
                }

                return RobotState.getBallsIn() < 3;
            }

            /**
             * Detects the artifact type based on hue values from both color sensors.
             * @param hue1 Hue value from the first color sensor.
             * @param hue2 Hue value from the second color sensor.
             */
            private Artifact detectArtifact(double hue1, double hue2) {
                if (isInRange(hue1, GREEN_HUE_MIN, GREEN_HUE_MAX) ||
                    isInRange(hue2, GREEN_HUE_MIN, GREEN_HUE_MAX)) {
                    return Artifact.GREEN;
                } else if (isInRange(hue1, PURPLE_HUE_MIN, PURPLE_HUE_MAX) ||
                           isInRange(hue2, PURPLE_HUE_MIN, PURPLE_HUE_MAX)) {
                    return Artifact.PURPLE;
                }
                return null;
            }

            private boolean isInRange(double value, double min, double max) {
                return value > min && value < max;
            }

            /**
             * Updates the indicator light for the given slot based on artifact type.
             * @param slot The index of the indicator light slot.
             * @param artifact The detected artifact type.
             */
            private void updateIndicatorLight(int slot, Artifact artifact) {
                double position = (artifact == Artifact.PURPLE) ? LIGHT_PURPLE : LIGHT_GREEN;
                switch (slot) {
                    case 0:
                        light2.setPosition(position);
                        break;
                    case 1:
                        light3.setPosition(position);
                        break;
                    case 2:
                        light4.setPosition(position);
                        break;
                }
            }
        };
    }

    /**
     * Creates an action that stops the intake motor.
     * <p>
     * This is a one-shot action that sets the intake motor power to 0
     * and immediately completes.
     * </p>
     *
     * @return An Action that stops the intake and returns {@code false} immediately.
     */
    public Action stopIntake() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0);
                    initialized = true;
                }
                return false;
            }
        };
    }

    /**
     * Creates an action that automatically aligns the robot to a target using the Limelight camera.
     * <p>
     * This action uses the Limelight's horizontal offset (tx) to rotate the robot
     * until it is aligned with the detected target. The action completes when the
     * robot is within the alignment threshold (tx between -1.9 and 1.6 degrees)
     * or after a 500ms timeout.
     * </p>
     *
     * @return An Action that returns {@code true} while aligning, and {@code false}
     *         once aligned or timed out.
     */
    public Action autoalign() {
        return new Action() {
            private boolean initialized = false;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                LLResult result = limelight.getLatestResult();
                double tx = result.getTx();

                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), -(tx/27.25 * 0.8)));
                updatePoseEstimate();

                light1.setPosition(.5);
                packet.put("tx", tx);
                packet.put("timer", timer.milliseconds());
                if ((tx < 1.6 && tx > -1.9) || timer.milliseconds() >= 500) {
                    light1.setPosition(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    /**
     * Creates an action that spins up the shooter motors to a target velocity.
     * <p>
     * This action sets the shooter motors to the specified velocity and waits
     * until the motors reach at least 1000 ticks/second before completing.
     * This ensures the shooter is ready before attempting to shoot.
     * </p>
     *
     * @param velocityTarget The target velocity in encoder ticks per second.
     * @return An Action that returns {@code true} while spinning up, and {@code false}
     *         once the shooter has reached sufficient speed.
     */
    public Action spinUpShooter(double velocityTarget) {
        return new Action() {
            private boolean initialized = false;
            private double velocity = velocityTarget;

            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                setShooterVelocity(velocity);

                double vel = lShooter.getVelocity();
                packet.put("vel", vel);
                return vel < 1_000;
            }
        };
    }

    /**
     * Creates an action that shoots a specified number of balls.
     * <p>
     * This action maintains the shooter at the target velocity and uses the
     * distance sensor to count balls as they pass through. The index and intake
     * motors are controlled based on shooter velocity to ensure consistent shots.
     * The action completes when all balls have been shot or after a 7-second timeout.
     * </p>
     * <p>
     * The shooter velocity is reduced by 600 ticks/second after completion to
     * conserve power while keeping the motors warm.
     * </p>
     *
     * @param ballsToShoot   The number of balls to shoot.
     * @param velocityTarget The target shooter velocity in encoder ticks per second.
     */
    public Action shootBall(int ballsToShoot, double velocityTarget) {
        return new Action() {
            private int ballsRemaining = ballsToShoot;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean lastSeenBall = false;

            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                lShooter.setVelocity(velocityTarget);
                rShooter.setVelocity(velocityTarget);

                boolean ballCurrentlySeen = distance1.getState();
                if (ballCurrentlySeen && !lastSeenBall) {
                    RobotState.setArtifacts(ballsRemaining - 1, null);
                    ballsRemaining--;
                }
                lastSeenBall = ballCurrentlySeen;

                if (ballsRemaining <= 0 || timer.milliseconds() >= 4000) {
                    index.setPower(0);
                    intake.setPower(0);
                    light1.setPosition(0);
                    RobotState.setBallsIn(0);
                    lShooter.setVelocity(velocityTarget - 600);
                    return false;
                }

                double lVel = Math.abs(lShooter.getVelocity());
                double rVel = Math.abs(rShooter.getVelocity());



                // Check if both shooter wheels are within the target velocity range
                if (lVel > (velocityTarget-50) && lVel < (velocityTarget+65) && rVel > (velocityTarget-50) && rVel < (velocityTarget+65)) {
                    index.setPower(1.0);
                    intake.setPower(0.7);
                    light1.setPosition(0.7);
                } else {
                    index.setPower(0);
                    intake.setPower(0.8);
                    light1.setPosition(0);
                }

                packet.put("ballsRemaining", ballsRemaining);
                packet.put("ballCurrentlySeen", ballCurrentlySeen);
                packet.put("lastSeenBall", lastSeenBall);
                packet.put("lShooterVelocity", lVel);
                packet.put("rShooterVelocity", rVel);
                return true;
            }
        };
    }
}
