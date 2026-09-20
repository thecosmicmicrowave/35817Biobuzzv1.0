package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.math.Pose;

import dev.nextftc.control.util.InterpolatingMap;
import kotlin.jvm.functions.Function3;

/**
 * Field geometry and tuning constants.
 *
 * Every value here is a placeholder until a tuning session fills it in. Tuning should only
 * ever need to edit this file — the subsystems read from it and hold no magic numbers of
 * their own.
 */
public final class Constants {

    private Constants() {
    }

    // ---------------------------------------------------------------- field geometry

    /** Field coordinate the shooter aims at. Heading is unused. */
    public static final Pose GOAL = new Pose(72, 144, 0);

    /** Fallback starting pose for OpModes that do not specify one. */
    public static final Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    // ---------------------------------------------------------------- turret

    /** Turret travel limits, in degrees relative to robot forward. */
    public static final double TURRET_MIN_DEG = -90;
    public static final double TURRET_MAX_DEG = 90;
    public static final double TURRET_CENTER_DEG = 0;

    /** Servo position commanded at {@link #TURRET_CENTER_DEG}. */
    public static final double TURRET_CENTER_POSITION = 0.5;

    /** Servo position change per degree of turret rotation. */
    public static final double TURRET_POSITION_PER_DEG = 0.5 / 90.0;

    /**
     * How long after a large commanded move the turret is assumed to have arrived.
     * Positional servos give no feedback, so this is an assumption, not a measurement.
     */
    public static final double TURRET_SETTLE_MS = 150;

    /** A commanded change smaller than this does not restart the settle timer. */
    public static final double TURRET_MOVE_EPSILON_DEG = 2.0;

    /** Degrees per loop of manual turret trim at full stick. */
    public static final double TURRET_TRIM_DEG_PER_LOOP = 1.5;

    /** Degrees the turret trim shifts per press of the trim controls. */
    public static final double TURRET_TRIM_STEP_DEG = 1.0;

    // ---------------------------------------------------------------- hood

    public static final double HOOD_MIN_POSITION = 0.0;
    public static final double HOOD_MAX_POSITION = 1.0;
    public static final double HOOD_STOWED_POSITION = 0.0;

    /** Hood position change per press of the trim controls. */
    public static final double HOOD_TRIM_STEP = 0.01;

    // ---------------------------------------------------------------- flywheel

    /**
     * Encoder ticks per revolution of the flywheel motor, at the output shaft the encoder is
     * on. 28 is a bare REV/goBILDA motor with no gearbox, which is the usual shooter setup.
     * Wrong here means every RPM reading and the whole DISTANCE_TO_RPM table are wrong.
     */
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    /**
     * Flywheel velocity control gains. These default to zero in NextFTC, and with all-zero
     * gains the motor's velocity loop outputs exactly zero power — the flywheel would never
     * spin. kV is the dominant term: roughly 1 / free-speed-in-RPM, so full command at free
     * speed. Add kP only after kV alone gets close.
     */
    public static final double FLYWHEEL_KP = 0.0;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KS = 0.0;
    public static final double FLYWHEEL_KV = 1.0 / 6000.0;
    public static final double FLYWHEEL_KA = 0.0;

    /** Idle speed held between shots so the flywheel does not spin up from zero. */
    public static final double IDLE_RPM = 1000;

    /** How close to the setpoint counts as up to speed. */
    public static final double RPM_TOLERANCE = 100;

    /** How long the flywheel must stay within tolerance before {@code atSpeed()} is true. */
    public static final double AT_SPEED_DEBOUNCE_SECONDS = 0.1;

    // ---------------------------------------------------------------- gate

    public static final double GATE_OPEN_POSITION = 0.6;
    public static final double GATE_CLOSED_POSITION = 0.2;

    // ---------------------------------------------------------------- flower arm

    public static final double FLOWER_STOWED_POSITION = 0.1;
    public static final double FLOWER_DEPLOYED_POSITION = 0.8;

    // ---------------------------------------------------------------- intake powers

    public static final double STAGE1_INTAKE_POWER = 1.0;
    public static final double STAGE2_INTAKE_POWER = 1.0;
    public static final double STAGE1_EJECT_POWER = -1.0;
    public static final double STAGE2_EJECT_POWER = -1.0;
    public static final double STAGE2_FEED_POWER = 1.0;

    // ---------------------------------------------------------------- timing

    /** How long the gate stays open to release a single artifact. */
    public static final double FEED_TIME_MS = 250;

    /** How long both stages run when emptying the robot into the goal. */
    public static final double VOLLEY_TIME_MS = 2000;

    /** Cap on waiting for the flywheel to reach speed, so a bad tune cannot stall a match. */
    public static final double SPINUP_TIMEOUT_MS = 2000;

    // ---------------------------------------------------------------- drivetrain
    //
    // Drivetrain motor names and directions live in pedro/Constants.java, not here: Pedro's
    // Follower owns those motors.

    // ---------------------------------------------------------------- lookup tables

    /**
     * Linear interpolation between the two bracketing table entries.
     * The cast disambiguates the {@code Function2} and {@code Function3} constructor overloads.
     */
    private static final Function3<Double, Double, Double, Double> LERP =
            (lower, upper, t) -> lower + (upper - lower) * t;

    /** Distance to the goal (inches) to flywheel speed (RPM). Placeholder values. */
    public static final InterpolatingMap<Double> DISTANCE_TO_RPM = new InterpolatingMap<>(LERP);

    /** Distance to the goal (inches) to hood servo position. Placeholder values. */
    public static final InterpolatingMap<Double> DISTANCE_TO_HOOD = new InterpolatingMap<>(LERP);

    static {
        // Placeholders so the maps are never empty. Replace wholesale during tuning:
        // shoot from a measured distance, adjust until it scores, record the pair.
        DISTANCE_TO_RPM.put(24.0, 2000.0);
        DISTANCE_TO_RPM.put(120.0, 3500.0);

        DISTANCE_TO_HOOD.put(24.0, 0.2);
        DISTANCE_TO_HOOD.put(120.0, 0.8);
    }

    // ---------------------------------------------------------------- helpers

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Wraps an angle in degrees to [-180, 180). */
    public static double normalizeDeg(double degrees) {
        double wrapped = (degrees + 180) % 360;
        if (wrapped < 0) {
            wrapped += 360;
        }
        return wrapped - 180;
    }
}
