package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.nextftc.control.filters.Debouncer;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Flywheel, hood, and turret.
 *
 * <p>Aiming is pose-driven: {@link #autoAim()} runs as the default command and recomputes the
 * turret angle and hood position from the current Pedro pose every loop, so the turret is
 * already tracking the goal by the time the driver asks for a shot.
 *
 * <p>{@link #targetRpm} is the single source of truth for flywheel speed, and every command
 * that changes it pushes it to the motor through {@link #applyFlywheel()}. That write is
 * idempotent, so {@code autoAim()} refreshing it every loop does not fight the spin-up
 * commands.
 *
 * <p>The spin-up commands must write the motor themselves rather than leaving it to
 * {@code autoAim()}: Ivy's {@code Sequential} unions its children's requirements and holds
 * them for the whole sequence, so a shot sequence owns the shooter from start to finish and
 * the default command is suspended throughout. If {@code autoAim()} were the only writer, the
 * flywheel would never spin up during a shot and every shot would hit its timeout.
 */
public class Shooter implements Mechanism {

    /**
     * Constructed with an explicit angle-per-count. NextFTC defaults it to one radian per
     * encoder tick, which would make every velocity reading meaningless.
     */
    private final NextMotor flywheel = new NextMotor(
            "flywheel", Units.Rotations.of(1.0 / Constants.FLYWHEEL_TICKS_PER_REV));
    private final NextServo hood = new NextServo("hood");
    private final NextServo turretLeft = new NextServo("turretLeft");
    private final NextServo turretRight = new NextServo("turretRight");

    private final Supplier<Pose> poseSupplier;

    private final Debouncer atSpeedDebouncer =
            new Debouncer(Constants.AT_SPEED_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);

    /** Single source of truth for flywheel speed. Pushed to the motor by {@link #applyFlywheel()}. */
    private double targetRpm = 0;

    /** When armed, {@code autoAim()} keeps {@link #targetRpm} synced to the distance table. */
    private boolean armed = false;

    private double commandedTurretDeg = Constants.TURRET_CENTER_DEG;
    private double commandedHoodPosition = Constants.HOOD_STOWED_POSITION;
    private boolean atSpeed = false;

    /**
     * Driver trim, applied on top of the pose-derived setpoints inside {@link #autoAim()}.
     * Trimming adjusts tracking rather than replacing it, so the turret keeps following the
     * goal and the trim survives across shots.
     */
    private double turretOffsetDeg = 0;
    private double hoodOffset = 0;

    /**
     * Turret settle tracking. {@code settledTargetDeg} is the angle the timer was last started
     * for; the timer restarts once the commanded angle has drifted a meaningful distance from
     * it. Comparing against the per-call delta instead would never trip, because
     * {@link #autoAim()} moves the turret a fraction of a degree per loop.
     */
    private double settledTargetDeg = Constants.TURRET_CENTER_DEG;
    private long lastTurretMoveMs = 0;

    /**
     * How long the current move is expected to take. Scales with the size of the move: at
     * roughly 755 degrees/s a small correction settles almost instantly, but a half-turn swing
     * takes a meaningful fraction of a second, and a fixed allowance would either stall every
     * small correction or fire mid-swing on a large one.
     */
    private double settleDurationMs = 0;

    /** True while {@link #manualTurret(DoubleSupplier)} owns the turret, suppressing tracking. */
    private boolean manualOverride = false;

    public Shooter(Supplier<Pose> poseSupplier) {
        this.poseSupplier = poseSupplier;
        turretRight.setDirection(NextMotor.Direction.REVERSE);

        // NextFTC defaults every velocity gain to zero, and the velocity loop computes
        // pid.calculate(...) + ff.calculate(...) — with all-zero gains the flywheel would be
        // commanded to exactly zero power no matter what setpoint it is given.
        MotorVelocityConstants velocity = flywheel.getVelocityConstants();
        velocity.setKP(Constants.FLYWHEEL_KP);
        velocity.setKI(Constants.FLYWHEEL_KI);
        velocity.setKD(Constants.FLYWHEEL_KD);
        velocity.setKS(Constants.FLYWHEEL_KS);
        velocity.setKV(Constants.FLYWHEEL_KV);
        velocity.setKA(Constants.FLYWHEEL_KA);
    }

    // ---------------------------------------------------------------- lifecycle

    /**
     * Returns the shooter to a known state. The robot and its mechanisms are constructed once
     * at event-loop creation and outlive individual OpMode runs, so cached state has to be
     * cleared explicitly or it carries over from the previous run.
     */
    public void reset() {
        armed = false;
        manualOverride = false;
        turretOffsetDeg = 0;
        hoodOffset = 0;
        targetRpm = 0;
        atSpeed = false;
        atSpeedDebouncer.calculate(false);
        settledTargetDeg = Constants.TURRET_CENTER_DEG;
        applyTurret(Constants.TURRET_CENTER_DEG);
        applyHood(Constants.HOOD_STOWED_POSITION);
        applyFlywheel();
    }

    @Override
    public void periodic() {
        atSpeed = atSpeedDebouncer.calculate(withinRpmTolerance());
    }

    @Override
    public Command getDefaultCommand() {
        return autoAim();
    }

    // ---------------------------------------------------------------- state

    public double distanceToGoal() {
        Pose pose = poseSupplier.get();
        double dx = Constants.GOAL.x() - pose.x();
        double dy = Constants.GOAL.y() - pose.y();
        return Math.hypot(dx, dy);
    }

    /** Bearing from the robot to the goal, in degrees relative to robot forward. */
    public double bearingToGoal() {
        Pose pose = poseSupplier.get();
        double dx = Constants.GOAL.x() - pose.x();
        double dy = Constants.GOAL.y() - pose.y();
        double fieldBearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        return Constants.normalizeDeg(fieldBearingDeg - Math.toDegrees(pose.heading()));
    }

    public boolean inTurretRange(double degrees) {
        return degrees >= Constants.TURRET_MIN_DEG && degrees <= Constants.TURRET_MAX_DEG;
    }

    public double currentRpm() {
        return flywheel.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double targetRpm() {
        return targetRpm;
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    public boolean isArmed() {
        return armed;
    }

    /** Commanded motor power, whatever control mode produced it. */
    public double flywheelThrottle() {
        return flywheel.getThrottle();
    }

    /**
     * Cuts flywheel power immediately, without going through a command. For OpMode teardown,
     * where the scheduler is being reset and a command would never execute.
     */
    public void coastFlywheel() {
        targetRpm = 0;
        armed = false;
        flywheel.setThrottle(0);
    }

    /** Flywheel current draw in amps. A stalled or rubbing flywheel shows up here first. */
    public double flywheelCurrentAmps() {
        return flywheel.getCurrent().into(Units.Amperes);
    }

    /**
     * Whether the turret has had time to reach its commanded angle. Positional servos report
     * nothing, so this is elapsed time since the last large commanded move — if the turret is
     * bumped or stalls, this still reads true.
     */
    public boolean turretSettled() {
        return System.currentTimeMillis() - lastTurretMoveMs >= settleDurationMs;
    }

    public boolean readyToShoot() {
        return targetRpm > 0 && atSpeed() && turretSettled();
    }

    public double commandedTurretDeg() {
        return commandedTurretDeg;
    }

    public double commandedHoodPosition() {
        return commandedHoodPosition;
    }

    // ---------------------------------------------------------------- commands

    /**
     * Default command. Tracks the goal and keeps the flywheel setpoint refreshed. Manual
     * commands and shot sequences preempt it through the normal requirement conflict, and it
     * resumes automatically when they finish.
     */
    public Command autoAim() {
        return infinite(() -> {
            double distance = distanceToGoal();

            // Only the turret is surrendered to manual control; the hood keeps tracking.
            if (!manualOverride) {
                applyTurret(Constants.clamp(bearingToGoal() + turretOffsetDeg,
                        Constants.TURRET_MIN_DEG, Constants.TURRET_MAX_DEG));
            }
            applyHood(Constants.clamp(
                    lookUp(Constants.DISTANCE_TO_HOOD, distance, commandedHoodPosition) + hoodOffset,
                    Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
            if (armed) {
                targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distance, targetRpm);
            }
            applyFlywheel();
        });
    }

    /** Arms the shooter so {@code autoAim()} tracks the mapped RPM for the current distance. */
    public Command spinUp() {
        return instant(() -> {
            armed = true;
            targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distanceToGoal(), targetRpm);
            applyFlywheel();
        });
    }

    /** Holds a fixed speed regardless of pose. Used by autonomous and for pit testing. */
    public Command spinUpTo(double rpm) {
        return instant(() -> {
            armed = false;
            targetRpm = rpm;
            applyFlywheel();
        });
    }

    /** Drops to idle rather than zero, so the next shot does not wait for a full spin-up. */
    public Command idleFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = Constants.IDLE_RPM;
            applyFlywheel();
        });
    }

    public Command stopFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = 0;
            applyFlywheel();
        });
    }

    /**
     * Direct open-loop control of all three shooter actuators, for bring-up and calibration.
     *
     * <p>The flywheel runs on raw throttle rather than a velocity setpoint, which switches the
     * motor's control mode away from velocity so the untuned velocity loop cannot interfere.
     * The turret and hood take raw servo positions, bypassing the degree mapping in
     * {@link #turretTo(double)} — that mapping is what calibration establishes, so it cannot be
     * used to find itself.
     *
     * <p>Infinite and requiring this mechanism, so while scheduled it owns the shooter:
     * {@code autoAim()} stays suspended and nothing else writes these actuators. That matters
     * during bring-up, because {@code autoAim()} is the default command and would otherwise
     * drive the turret to its computed bearing using calibration constants that are still
     * placeholders.
     */
    public Command manualControl(
            DoubleSupplier power, DoubleSupplier turretPosition, DoubleSupplier hoodPosition) {
        return infinite(() -> {
            flywheel.setThrottle(Constants.clamp(power.getAsDouble(), -1.0, 1.0));

            double turret = Constants.clamp(turretPosition.getAsDouble(), 0, 1);
            // turretRight is reversed in the constructor, so both take the same value.
            turretLeft.setPosition(turret);
            turretRight.setPosition(turret);

            hood.setPosition(Constants.clamp(hoodPosition.getAsDouble(), 0, 1));
        });
    }

    /**
     * The turret angle the current constants would infer from a raw servo position. Lets a
     * calibration OpMode show whether the constants agree with the physical turret yet.
     */
    public static double degreesForTurretPosition(double position) {
        return Constants.TURRET_CENTER_DEG
                + (position - Constants.TURRET_CENTER_POSITION) / Constants.TURRET_POSITION_PER_DEG;
    }

    /** Driver turret nudge. Preempts {@code autoAim()}, which resumes when this is released. */
    public Command manualTurret(DoubleSupplier input) {
        return infinite(() -> {
            manualOverride = true;
            double delta = input.getAsDouble() * Constants.TURRET_TRIM_DEG_PER_LOOP;
            applyTurret(Constants.clamp(commandedTurretDeg + delta,
                    Constants.TURRET_MIN_DEG, Constants.TURRET_MAX_DEG));
            applyFlywheel();
        }).setEnd(endCondition -> manualOverride = false);
    }

    public Command turretTo(double degrees) {
        return instant(() -> applyTurret(Constants.clamp(degrees,
                Constants.TURRET_MIN_DEG, Constants.TURRET_MAX_DEG)));
    }

    public Command centerTurret() {
        return turretTo(Constants.TURRET_CENTER_DEG);
    }

    /** Manual hood override. Also suppresses pose tracking of the hood while held. */
    public Command hoodTo(double position) {
        return instant(() -> applyHood(Constants.clamp(position,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION)));
    }

    /**
     * Biases the tracked hood position. An offset rather than a direct write, so pose tracking
     * continues and the trim carries across shots.
     */
    public Command trimHood(double delta) {
        return instant(() -> hoodOffset += delta);
    }

    /** Biases the tracked turret angle. */
    public Command trimTurret(double deltaDeg) {
        return instant(() -> turretOffsetDeg += deltaDeg);
    }

    /** Drops any manual override and clears the trims, returning to pure pose tracking. */
    public Command resumeAutoAim() {
        return instant(() -> {
            manualOverride = false;
            turretOffsetDeg = 0;
            hoodOffset = 0;
        });
    }

    // ---------------------------------------------------------------- hardware

    private void applyTurret(double degrees) {
        // Measured against the angle the settle timer was started for, not against the previous
        // call: autoAim() nudges the turret a fraction of a degree per loop, so a per-call
        // comparison would never exceed the epsilon and the turret would always read settled.
        double moveDeg = Math.abs(degrees - settledTargetDeg);
        if (moveDeg >= Constants.TURRET_MOVE_EPSILON_DEG) {
            settleDurationMs = Constants.TURRET_SETTLE_BASE_MS
                    + moveDeg * Constants.TURRET_SETTLE_MS_PER_DEG;
            settledTargetDeg = degrees;
            lastTurretMoveMs = System.currentTimeMillis();
        }
        commandedTurretDeg = degrees;

        double position = Constants.TURRET_CENTER_POSITION
                + (degrees - Constants.TURRET_CENTER_DEG) * Constants.TURRET_POSITION_PER_DEG;
        position = Constants.clamp(position, 0, 1);

        // turretRight is reversed in the constructor, so both take the same commanded position.
        turretLeft.setPosition(position);
        turretRight.setPosition(position);
    }

    /** Pushes {@link #targetRpm} to the motor. Idempotent, so every writer may call it. */
    private void applyFlywheel() {
        flywheel.setVelocitySetpoint(Units.RotationsPerMinute.of(targetRpm));
    }

    private void applyHood(double position) {
        commandedHoodPosition = position;
        hood.setPosition(position);
    }

    private boolean withinRpmTolerance() {
        if (targetRpm <= 0) {
            return false;
        }
        return Math.abs(currentRpm() - targetRpm) <= Constants.RPM_TOLERANCE;
    }

    /** Reads a lookup table, falling back to {@code fallback} if the table has no entries. */
    private static double lookUp(
            dev.nextftc.control.util.InterpolatingMap<Double> map, double key, double fallback) {
        if (map.isEmpty()) {
            return fallback;
        }
        Double value = map.get(key);
        return value == null ? fallback : value;
    }
}
