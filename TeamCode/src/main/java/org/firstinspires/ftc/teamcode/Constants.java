package org.firstinspires.ftc.teamcode;

public final class Constants {
    private Constants() {
        throw new UnsupportedOperationException("Utility class");
    }

    public static final class Intake {
        public static final double GREEN_HUE_MIN = 48.0;
        public static final double GREEN_HUE_MAX = 165.0;
        public static final double PURPLE_HUE_MIN = 211.0;
        public static final double PURPLE_HUE_MAX = 338.0;
        public static final double LIGHT_PURPLE = 0.7;
        public static final double LIGHT_GREEN = 0.5;
        public static final int MAX_BALLS = 3;

        private Intake() {
            throw new UnsupportedOperationException("Utility class");
        }
    }

    public static final class Shooter {
        public static final double TARGET_FEET = 3.0 + (8.0 / 12.0);
        public static final double LAUNCH_ANGLE_DEG = 61.0;
        public static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
        public static final double GRAVITY_FT_S2 = 32.2;
        public static final double SHOOTER_WHEEL_DIAMETER_FT = 0.315;
        public static final double RPM_EMPIRICAL_FACTOR = 0.705;
        public static final double RPM_MAGIC_CONSTANT = 120.0;
        public static final double DEFAULT_FALLBACK_VELOCITY_FT_S = 17.0;
        public static final double MANUAL_DEFAULT_RPM = 800.0;
        public static final double RPM_STEP = 50.0;
        public static final double GEAR_RATIO = 30.0 / 24.0;
        public static final double COUNTS_PER_MOTOR_REV = 28.0;
        public static final double TICKS_PER_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
        public static final double PID_P = 19.0;
        public static final double PID_I = 0.1;
        public static final double PID_D = 0.0;
        public static final double PID_F = 26.0;

        private Shooter() {
            throw new UnsupportedOperationException("Utility class");
        }
    }

    public static final class Vision {
        public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0;
        public static final double LIMELIGHT_HEIGHT_IN = 12.5;
        public static final double GOAL_HEIGHT_IN = 29.5;
        public static final double DISTANCE_OFFSET_FT = 0.3;
        public static final int LIMELIGHT_POLL_RATE_HZ = 70;
        public static final long THREAD_SLEEP_MS = 5L;

        private Vision() {
            throw new UnsupportedOperationException("Utility class");
        }
    }

    public static final class Drive {
        public static final double DEAD_BAND = 0.05;
        public static final double ROTATION_SCALE = 0.6;
        public static final double TRANSLATION_SCALE = 0.6;
        public static final double TRANSLATION_GAIN = 1.33;
        public static final double LIMELIGHT_TURN_DIVISOR = 27.25;

        private Drive() {
            throw new UnsupportedOperationException("Utility class");
        }
    }

    public static final class TeleOp {
        public static final double RIGHT_TRIGGER_THRESHOLD = 0.2;
        public static final long DEBOUNCE_MS = 300L;
        public static final int PRISM_STRIP_LENGTH = 29;

        private TeleOp() {
            throw new UnsupportedOperationException("Utility class");
        }
    }
}
