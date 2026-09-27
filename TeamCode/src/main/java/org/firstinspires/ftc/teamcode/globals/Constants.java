package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import dev.nextftc.control.util.InterpolatingMap;
import kotlin.jvm.functions.Function3;

/**
 * Field geometry and tuning constants.
 *
 * <p>Every value here is a placeholder until a tuning session fills it in. Tuning should only
 * ever need to edit this file — the subsystems read from it and hold no magic numbers of
 * their own.
 *
 * <h2>Live tuning with Panels</h2>
 * <p>{@code @Configurable} exposes this class in the Panels Configurables tab. Panels only picks
 * up fields that are {@code static} and <b>not</b> {@code final}, so every tunable value here is
 * deliberately non-final. That also matters for correctness, not just visibility: javac inlines
 * {@code static final} primitives into every call site at compile time, so even a reflective edit
 * to a final constant would never be seen by the code reading it.
 *
 * <p>Panels edits are live but not saved. Once a value is right, copy it back into this file.
 *
 * <p>Values that are computed from others are methods rather than fields, so they recompute on
 * every read and stay consistent while their inputs are being edited. Physical constants that
 * never change at runtime (gear tooth counts, the fallback start pose, the lookup tables) stay
 * {@code final} and do not appear in Panels.
 */
@Configurable
public final class Constants {

    private Constants() {
    }

    // ---------------------------------------------------------------- field geometry

    /** Field coordinate the shooter aims at, in inches in Pedro's field frame. */
    public static double GOAL_X = 72;
    public static double GOAL_Y = 144;

    /** Fallback starting pose for OpModes that do not specify one. */
    public static final Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    // ---------------------------------------------------------------- turret geometry
    //
    // Two SWYFT Balance servos, each with a 60t gear, both driving a 20t gear on a shared shaft.
    // A 65t gear on that shaft drives the 169t turret ring gear:
    //
    //     turret degrees per servo degree = (60 / 20) * (65 / 169) = 15 / 13 ~= 1.154
    //
    // The turret turns slightly MORE than the servo. Everything below is derived from these
    // numbers, so a hardware change only means editing the teeth counts or the servo travel.

    /**
     * Rotation of one turret servo across its full 0..1 position range.
     *
     * <p>SWYFT servos ship at 270 degrees. 320 requires reprogramming each servo with the SWYFT
     * Servo Programmer. If these servos are still at 270, set this to 270 — at 320 the turret
     * reaches every bearing, but at 270 it only covers +/-156 degrees and has a blind spot behind
     * the robot.
     *
     * <p>The derived calibration also assumes the servo's full travel maps onto the SDK's 0..1
     * position range, which only holds when {@link #TURRET_SERVO_PWM_MIN_US} and
     * {@link #TURRET_SERVO_PWM_MAX_US} match what the servo was programmed with.
     */
    public static double TURRET_SERVO_TRAVEL_DEG = 320;

    /**
     * Pulse widths the turret servos were programmed to span their full travel with, from the
     * Min Signal and Max Signal fields of the SWYFT Servo Programmer.
     *
     * <p>These must match the programmer. The SDK's default servo range is 600-2400 us, so left
     * at the default a servo programmed for 500-2500 us only reaches 90% of its travel: 288 of
     * 320 degrees, which after gearing leaves the turret about 28 degrees short of a full
     * rotation, and makes every commanded angle land ~10% short.
     *
     * <p>Final rather than tunable: these are applied once when the servos are configured and
     * describe how the hardware was programmed, not something to tweak live.
     */
    public static final double TURRET_SERVO_PWM_MIN_US = 500;
    public static final double TURRET_SERVO_PWM_MAX_US = 2500;

    /**
     * Software direction for each turret servo. The two servos must end up turning the turret the
     * same way, and their net direction is this flag combined with the Reverse direction toggle
     * in the SWYFT Servo Programmer — a servo reversed in both places is back to forward. If both
     * servos were programmed identically, at most one of these should be true, and whether one
     * needs to be depends on whether the servos are mounted mirrored.
     */
    public static final boolean TURRET_LEFT_REVERSED = false;
    public static final boolean TURRET_RIGHT_REVERSED = false;

    public static final double TURRET_SERVO_GEAR_TEETH = 60;
    public static final double TURRET_SHAFT_INPUT_GEAR_TEETH = 20;
    public static final double TURRET_SHAFT_OUTPUT_GEAR_TEETH = 65;
    public static final double TURRET_RING_GEAR_TEETH = 169;

    /** Turret degrees per servo degree. 15/13 for the current gearing. */
    public static double turretDegPerServoDeg() {
        return (TURRET_SERVO_GEAR_TEETH / TURRET_SHAFT_INPUT_GEAR_TEETH)
                * (TURRET_SHAFT_OUTPUT_GEAR_TEETH / TURRET_RING_GEAR_TEETH);
    }

    /** Total turret rotation across the servo's full range. ~369.2 degrees at 320 degrees of servo travel. */
    public static double turretTravelDeg() {
        return TURRET_SERVO_TRAVEL_DEG * turretDegPerServoDeg();
    }

    /** Servo position change per degree of turret rotation. */
    public static double turretPositionPerDeg() {
        return 1.0 / turretTravelDeg();
    }

    // ---------------------------------------------------------------- turret calibration

    public static final double TURRET_CENTER_DEG = 0;

    /**
     * Servo position at which the turret points straight forward. Depends on how the gears were
     * meshed during assembly, not on the gear ratio — find it with the Shooter Test.
     */
    public static double TURRET_CENTER_POSITION = 0.5;

    /** Kept clear of each physical end of travel so the servos never stall against a stop. */
    public static double TURRET_LIMIT_MARGIN_DEG = 2.0;

    /**
     * Turret travel limits, in degrees relative to robot forward. Derived from where the servo's
     * physical ends of travel land given the center position, pulled in by the margin, and capped
     * at +/-180 since no bearing needs more. At 320 degrees of servo travel and a centered 0.5,
     * this is the full +/-180.
     */
    public static double turretMinDeg() {
        return Math.max(-180, TURRET_CENTER_DEG - TURRET_CENTER_POSITION / turretPositionPerDeg()
                + TURRET_LIMIT_MARGIN_DEG);
    }

    public static double turretMaxDeg() {
        return Math.min(180, TURRET_CENTER_DEG + (1 - TURRET_CENTER_POSITION) / turretPositionPerDeg()
                - TURRET_LIMIT_MARGIN_DEG);
    }

    // ---------------------------------------------------------------- turret timing

    /**
     * Fixed settle allowance for any turret move: acceleration, deceleration, and gear backlash.
     * Positional servos give no feedback, so settling is an estimate, never a measurement.
     */
    public static double TURRET_SETTLE_BASE_MS = 60;

    /**
     * Additional settle allowance per degree of turret travel. A SWYFT Balance runs 109 RPM
     * unloaded, which is 654 degrees/s at the servo and ~755 degrees/s at the turret — about
     * 1.3 ms per turret degree. 2.0 leaves margin for the turret's load.
     */
    public static double TURRET_SETTLE_MS_PER_DEG = 2.0;

    /** A commanded change smaller than this does not restart the settle timer. */
    public static double TURRET_MOVE_EPSILON_DEG = 2.0;

    /** Degrees per loop of manual turret trim at full stick. */
    public static double TURRET_TRIM_DEG_PER_LOOP = 1.5;

    /** Degrees the turret trim shifts per press of the trim controls. */
    public static double TURRET_TRIM_STEP_DEG = 1.0;

    /** Turret speed at full stick in the Shooter Test, in turret degrees per second. */
    public static double TURRET_STICK_DEG_PER_SEC = 700;

    /** Stick input smaller than this is ignored, so a resting stick does not creep the turret. */
    public static double STICK_DEADZONE = 0.05;

    // ---------------------------------------------------------------- hood

    public static double HOOD_MIN_POSITION = 0.0;
    public static double HOOD_MAX_POSITION = 0.4;
    public static double HOOD_STOWED_POSITION = 0.0;

    /** Hood position change per press of the trim controls. */
    public static double HOOD_TRIM_STEP = 0.01;

    /** Hood speed at full stick in the Shooter Test, in servo position per second. */
    public static double HOOD_STICK_POSITION_PER_SEC = 0.5;

    // ---------------------------------------------------------------- flywheel

    /**
     * Encoder ticks per revolution of the flywheel motor, at the output shaft the encoder is
     * on. 28 is a bare REV/goBILDA motor with no gearbox, which is the usual shooter setup.
     * Wrong here means every RPM reading and the whole DISTANCE_TO_RPM table are wrong.
     */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    /**
     * Flywheel velocity control gains. These default to zero in NextFTC, and with all-zero
     * gains the motor's velocity loop outputs exactly zero power — the flywheel would never
     * spin. kV is the dominant term: roughly 1 / free-speed-in-RPM, so full command at free
     * speed. Add kP only after kV alone gets close.
     */
    public static double FLYWHEEL_KP = 0.0;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KS = 0.0;
    public static double FLYWHEEL_KV = 1.0 / 6000.0;
    public static double FLYWHEEL_KA = 0.0;

    /** Idle speed held between shots so the flywheel does not spin up from zero. */
    public static double IDLE_RPM = 1000;

    /** How close to the setpoint counts as up to speed. */
    public static double RPM_TOLERANCE = 100;

    /** How long the flywheel must stay within tolerance before {@code atSpeed()} is true. */
    public static double AT_SPEED_DEBOUNCE_SECONDS = 0.1;

    // ---------------------------------------------------------------- gate

    public static double GATE_OPEN_POSITION = 0.6;
    public static double GATE_CLOSED_POSITION = 0.2;

    // ---------------------------------------------------------------- flower arm

    public static double FLOWER_STOWED_POSITION = 0.1;
    public static double FLOWER_DEPLOYED_POSITION = 0.8;

    // ---------------------------------------------------------------- intake powers

    public static double STAGE1_INTAKE_POWER = 1.0;
    public static double STAGE2_INTAKE_POWER = 1.0;
    public static double STAGE1_EJECT_POWER = -1.0;
    public static double STAGE2_EJECT_POWER = -1.0;
    public static double STAGE2_FEED_POWER = 1.0;

    // ---------------------------------------------------------------- timing

    /** How long the gate stays open to release a single artifact. */
    public static double FEED_TIME_MS = 250;

    /** How long both stages run when emptying the robot into the goal. */
    public static double VOLLEY_TIME_MS = 2000;

    /** Cap on waiting for the flywheel to reach speed, so a bad tune cannot stall a match. */
    public static double SPINUP_TIMEOUT_MS = 2000;

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
