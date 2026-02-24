package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.Arrays;
import java.util.List;

/**
 * A parallel action that completes as soon as ANY one of its actions completes.
 * This is useful for stopping a movement early when a condition (like ball count) is met.
 *
 * Unlike ParallelAction which waits for ALL actions to complete,
 * RaceParallelAction "races" the actions and finishes when the first one ends.
 */
public class RaceParallelAction implements Action {
    private final List<Action> actions;

    /**
     * Creates a new RaceParallelAction.
     *
     * @param actions the actions to race
     */
    public RaceParallelAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        for (Action action : actions) {
            // run() returns true if the action is still running
            if (!action.run(packet)) {
                // One action finished, so we're done (race is over)
                return false;
            }
        }
        // All actions are still running
        return true;
    }
}