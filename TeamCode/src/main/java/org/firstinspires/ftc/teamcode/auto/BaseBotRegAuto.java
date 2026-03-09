package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pipeline;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Basebot Reg Auto")
public class BaseBotRegAuto extends OpMode {
    private RobotHardware robot;
    private final TimerEx timer = new TimerEx(30, TimeUnit.SECONDS);

    private double cycles = 4;
    private boolean close;
    private boolean pattern;
    private Alliance alliance;
    private double sideMultiplier;

    private double facingGate;
    private Pose2d farShot;
    private Pose2d closeShot;
    private Pose2d farStart;
    private Pose2d closeStart;
    private Pose2d PPG;
    private Pose2d PGP;
    private Pose2d GPP;

    private Pose2d start;
    private Pose2d targetShot;
    private Pose2d lastIntake;

    private boolean closeChosen;
    private boolean lastDpadDown;
    private boolean lastDpadUp;

    @Override
    public void init() {
        closeChosen = false;
        lastDpadDown = false;
        lastDpadUp = false;
    }

    @Override
    public void init_loop() {
        if (alliance == null) {
            telemetry.addLine("Selected Side: Press X for Blue, B for Red");
        } else {
            telemetry.addData("Selected Side", alliance);
        }

        if (!closeChosen) {
            telemetry.addLine("Close Shot: Press A for true, Y for false");
        } else {
            telemetry.addData("Close Shot", close);
        }

        telemetry.addData("Cycles", cycles);
        telemetry.addData("pattern first", pattern);
        telemetry.update();

        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.dpad_right) pattern = true;
        if (gamepad1.dpad_left) pattern = false;

        if (gamepad1.a) {
            close = true;
            closeChosen = true;
        }

        if (gamepad1.y) {
            close = false;
            closeChosen = true;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            cycles = Math.max(cycles - 1, 0);
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            cycles = Math.min(cycles + 1, 4);
        }

        lastDpadDown = gamepad1.dpad_down;
        lastDpadUp = gamepad1.dpad_up;
    }

    @Override
    public void start() {
        if (alliance == null) {
            alliance = Alliance.BLUE;
        }

        if (alliance.equals(Alliance.BLUE)) {
            farStart = new Pose2d(new Vector2d(64.75, -7.125), Math.toRadians(180));
        } else {
            farStart = new Pose2d(new Vector2d(64.75, 9), Math.toRadians(180));
        }
        start = close ? closeStart : farStart;

        robot = new RobotHardware(hardwareMap, start);
        robot.limelight.pipelineSwitch(0);
        LLResult result = robot.limelight.getLatestResult();

        int fiducialId = 0;
        if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            fiducialId = result.getFiducialResults().get(0).getFiducialId();
        }

        if (alliance.equals(Alliance.BLUE)) {
            robot.limelight.pipelineSwitch(Pipeline.BLUE_PIPELINE.getValue());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            facingGate = 270;
        } else {
            robot.limelight.pipelineSwitch(Pipeline.RED_PIPELINE.getValue());
            robot.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
            facingGate = 90;
        }

        sideMultiplier = alliance.getValue();

        PPG = new Pose2d(new Vector2d(33, 25 * sideMultiplier), Math.toRadians(facingGate));
        PGP = new Pose2d(new Vector2d(12, 25 * sideMultiplier), Math.toRadians(facingGate));
        GPP = new Pose2d(new Vector2d(-10.5, 25 * sideMultiplier), Math.toRadians(facingGate));
        farShot = new Pose2d(new Vector2d(60, 8 * sideMultiplier), Math.toRadians(alliance.equals(Alliance.BLUE) ? 200 : 155));
        closeShot = new Pose2d(new Vector2d(-10, 10 * sideMultiplier), Math.toRadians(225));
        farStart = new Pose2d(new Vector2d(64.75, 7.125 * sideMultiplier), Math.toRadians(180));
        closeStart = new Pose2d(new Vector2d(-58, 50 * sideMultiplier), Math.toRadians(45));

        switch (fiducialId) {
            case 21:
                lastIntake = GPP;
                break;
            case 22:
                lastIntake = PGP;
                break;
            case 23:
                lastIntake = PPG;
                break;
            default:
                lastIntake = close ? GPP : PPG;
                break;
        }

        targetShot = close ? closeShot : farShot;

        timer.start();
    }

    @Override
    public void loop() {
        // Intentionally empty per request: no runtime autonomous loop behavior.
    }

    @Override
    public void stop() {
        // No-op.
    }
}

