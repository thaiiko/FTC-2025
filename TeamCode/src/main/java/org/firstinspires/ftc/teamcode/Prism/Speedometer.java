package org.firstinspires.ftc.teamcode.Prism;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;


public class Speedometer {
    GoBildaPrismDriver prism;
    PrismAnimations.Solid solid = new PrismAnimations.Solid(new Color(42, 173, 77));
    PrismAnimations.Solid solid1 = new PrismAnimations.Solid(new Color(42, 173, 77));
    double oldVelocity;

    private final ElapsedTime timer = new ElapsedTime();


    public void speedAnim(double forwardVelocity) {
        prism.setStripLength(29);
        forwardVelocity = Math.abs(forwardVelocity);
        double oldVelocityDifference = Math.abs(forwardVelocity + oldVelocity);
        oldVelocity = forwardVelocity;

        solid.setStartIndex(0);
        solid.setStopIndex((int) (11 * forwardVelocity));
        solid.setBrightness(100);

        solid1.setStartIndex((int) (29-(11 * forwardVelocity)));
        solid1.setStopIndex(29);
        solid1.setBrightness(100);

        if (0.1 < forwardVelocity && forwardVelocity < 0.4) {
            solid.setPrimaryColor(Color.GREEN);
            solid1.setPrimaryColor(Color.GREEN);
        } else if (0.4 <= forwardVelocity && forwardVelocity < 0.8) {
            solid.setPrimaryColor(Color.YELLOW);
            solid1.setPrimaryColor(Color.YELLOW);
        } else if (0.8 <= forwardVelocity) {
            solid.setPrimaryColor(Color.RED);
            solid1.setPrimaryColor(Color.RED);
        }


        prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_0);
        prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_1);
        timer.reset();
    }
    public Speedometer(RobotHardware robot) {
        prism = robot.prism;
        // Insert animations once at initialization to avoid white flash on subsequent updates
        prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, solid1);
    }
}
