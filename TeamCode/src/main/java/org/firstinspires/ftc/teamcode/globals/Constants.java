package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import dev.nextftc.control.util.InterpolatingMap;
import kotlin.jvm.functions.Function3;

/** Tuning numbers + field geometry. */
@Configurable
public final class Constants {

    // Tunables: static, not final. Panels need it.

    private Constants() {
    }

    // ---------------------------------------------------------------- field geometry

    /** Field size, inches. Pedro field 0..144 both axes. */
    public static final double FIELD_SIZE_IN = 144;

    /** Goal spot, inches, BLUE alliance. Use goalX()/goalY(), they mirror for red. */
    public static double GOAL_X = 72;
    public static double GOAL_Y = 144;

    /** Start pose when OpMode give none. */
    public static final Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    // ---------------------------------------------------------------- turret geometry
    //
    // turret deg per servo deg = (60 / 20) * (65 / 169) = 15 / 13 ~= 1.154

    /** One turret servo full travel, degrees. */
    public static double TURRET_SERVO_TRAVEL_DEG = 320;

    /** Turret servo pulse range, us. Match SWYFT programmer. */
    public static final double TURRET_SERVO_PWM_MIN_US = 500;
    public static final double TURRET_SERVO_PWM_MAX_US = 2500;

    /** Turret servo directions. */
    public static final boolean TURRET_LEFT_REVERSED = false;
    public static final boolean TURRET_RIGHT_REVERSED = false;

    public static final double TURRET_SERVO_GEAR_TEETH = 60;
    public static final double TURRET_SHAFT_INPUT_GEAR_TEETH = 20;
    public static final double TURRET_SHAFT_OUTPUT_GEAR_TEETH = 65;
    public static final double TURRET_RING_GEAR_TEETH = 169;

    /** Turret deg per servo deg. 15/13 now. */
    public static double turretDegPerServoDeg() {
        return (TURRET_SERVO_GEAR_TEETH / TURRET_SHAFT_INPUT_GEAR_TEETH)
                * (TURRET_SHAFT_OUTPUT_GEAR_TEETH / TURRET_RING_GEAR_TEETH);
    }

    /** Total turret travel. ~369 deg at 320 servo. */
    public static double turretTravelDeg() {
        return TURRET_SERVO_TRAVEL_DEG * turretDegPerServoDeg();
    }

    /** Servo position per turret degree. */
    public static double turretPositionPerDeg() {
        return 1.0 / turretTravelDeg();
    }

    // ---------------------------------------------------------------- turret calibration

    public static final double TURRET_CENTER_DEG = 0;

    /** Servo position, turret straight forward. */
    public static double TURRET_CENTER_POSITION = 0.5;

    /** Gap kept from each end of travel. */
    public static double TURRET_LIMIT_MARGIN_DEG = 2.0;

    /** Turret limits, deg from robot forward. */
    public static double turretMinDeg() {
        return Math.max(-180, TURRET_CENTER_DEG - TURRET_CENTER_POSITION / turretPositionPerDeg()
                + TURRET_LIMIT_MARGIN_DEG);
    }

    public static double turretMaxDeg() {
        return Math.min(180, TURRET_CENTER_DEG + (1 - TURRET_CENTER_POSITION) / turretPositionPerDeg()
                - TURRET_LIMIT_MARGIN_DEG);
    }

    // ---------------------------------------------------------------- turret timing

    /** Base settle wait per turret move. */
    public static double TURRET_SETTLE_BASE_MS = 60;

    /** Extra settle wait per degree moved. */
    public static double TURRET_SETTLE_MS_PER_DEG = 2.0;

    /** Smaller move no restart settle timer. */
    public static double TURRET_MOVE_EPSILON_DEG = 2.0;

    /** manualTurret() deg per loop at full input. */
    public static double TURRET_TRIM_DEG_PER_LOOP = 1.5;

    /** Turret trim deg per press. */
    public static double TURRET_TRIM_STEP_DEG = 1.0;

    /** Shooter Test turret speed at full stick, deg/sec. */
    public static double TURRET_STICK_DEG_PER_SEC = 250;

    /** Stick below this ignored. */
    public static double STICK_DEADZONE = 0.05;

    // ---------------------------------------------------------------- hood

    public static double HOOD_MIN_POSITION = 0.0;
    public static double HOOD_MAX_POSITION = 0.45;
    public static double HOOD_STOWED_POSITION = 0.0;

    /** Hood trim per press. */
    public static double HOOD_TRIM_STEP = 0.01;

    /** Shooter Test hood speed at full stick, position/sec. */
    public static double HOOD_STICK_POSITION_PER_SEC = 0.5;

    // ---------------------------------------------------------------- flywheel

    /** Flywheel encoder ticks per rev. */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    /** Flywheel velocity gains. */
    public static double FLYWHEEL_KP = 0.0;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KS = 0.0;
    public static double FLYWHEEL_KV = 1.0 / 6000.0;
    public static double FLYWHEEL_KA = 0.0;

    /** Idle speed between shots. */
    public static double IDLE_RPM = 1000;

    /** This close to target = at speed. */
    public static double RPM_TOLERANCE = 100;

    /** Stay in tolerance this long before atSpeed() true. */
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

    /** Gate open time for one artifact. */
    public static double FEED_TIME_MS = 250;

    /** Both stages run time to empty robot. */
    public static double VOLLEY_TIME_MS = 2000;

    /** Max wait for flywheel speed. */
    public static double SPINUP_TIMEOUT_MS = 2000;

    // ---------------------------------------------------------------- drivetrain
    //
    // Drive motors set in pedro/Constants.java.

    // ---------------------------------------------------------------- lookup tables

    /** Blend between two table entries. */
    private static final Function3<Double, Double, Double, Double> LERP =
            (lower, upper, t) -> lower + (upper - lower) * t;

    /** Goal distance (inches) -> flywheel RPM. Placeholder. */
    public static final InterpolatingMap<Double> DISTANCE_TO_RPM = new InterpolatingMap<>(LERP);

    /** Goal distance (inches) -> hood position. Placeholder. */
    public static final InterpolatingMap<Double> DISTANCE_TO_HOOD = new InterpolatingMap<>(LERP);

    static {
        // Placeholders. Replace when tuning.
        DISTANCE_TO_RPM.put(24.0, 2000.0);
        DISTANCE_TO_RPM.put(120.0, 3500.0);

        DISTANCE_TO_HOOD.put(24.0, 0.2);
        DISTANCE_TO_HOOD.put(120.0, 0.8);
    }

    // ---------------------------------------------------------------- helpers

    /** Goal x for alliance. */
    public static double goalX(Alliance alliance) {
        return alliance == Alliance.RED ? FIELD_SIZE_IN - GOAL_X : GOAL_X;
    }

    /** Goal y for alliance. */
    public static double goalY(Alliance alliance) {
        return GOAL_Y;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Wrap degrees to [-180, 180). */
    public static double normalizeDeg(double degrees) {
        double wrapped = (degrees + 180) % 360;
        if (wrapped < 0) {
            wrapped += 360;
        }
        return wrapped - 180;
    }
}
