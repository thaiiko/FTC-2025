package org.firstinspires.ftc.teamcode;

import static java.lang.Runtime.getRuntime;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.jetbrains.annotations.NotNull;

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
    public DigitalChannel distance1;
//    public DistanceSensor distanceSensor;
//    public getPose() {
//        return super();
//    }

//    public Pose2d getPose() {
//        return this.pose;
//    }
    public RobotHardware(HardwareMap hardwareMap, Pose2d pose)  {
        super(hardwareMap, pose);

        distance1 = hardwareMap.get(DigitalChannel.class, "distance1");
        distance1.setMode(DigitalChannel.Mode.INPUT);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        light1 = hardwareMap.get(Servo.class, "light1");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");
        index = hardwareMap.get(DcMotorEx.class, "index");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);

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

    public void setShooterPower(double power) {
        lShooter.setPower(power);
        rShooter.setPower(power);
    }

    public double getVelocity() {
        return lShooter.getVelocity();
    }
    public void setShooterVelocity(double velocity) {
        lShooter.setVelocity(velocity);
        rShooter.setVelocity(velocity);
    }
    public Action startIntake() {
        return new IntakeStart();
    }

    public class IntakeStart implements Action {
        private boolean initalized = false;

        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            if (!initalized) {
                intake.setPower(1);
                initalized = true;
            }

            return false;
        }
    }
    public Action spinUpShooter() {
        return new Action () {
          private boolean initialized = false;

          @Override
          public boolean run(@NotNull TelemetryPacket packet) {
              if(!initialized) {
                  setShooterVelocity(1100);
                  initialized = true;
              }

              double vel =  lShooter.getVelocity();
              packet.put("vel", vel);
              return vel < 1_100;
          }
        };
    }
    public Action shootBall(int ballsToShoot) {
        return new Action() {
            private int balls = ballsToShoot;
            private boolean lastSeenBall = false;

            @Override
            public boolean run(@NotNull TelemetryPacket packet) {
                if (lShooter.getVelocity() > 1000 && lShooter.getVelocity() < 1140) {
                    index.setPower(0.8);
                    intake.setPower(1);
                    packet.put("debug balls", balls);

                    if (distance1.getState() && !lastSeenBall) {
                        balls--;
                        lastSeenBall = true;
                    } else {
                        lastSeenBall = false;
                    }

                    return balls >= 0;
                } else {
                    index.setPower(0);
                    intake.setPower(0);
                    return true;
                }
            }
        };
    }
}
