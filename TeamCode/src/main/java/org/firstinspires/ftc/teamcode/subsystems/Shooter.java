package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.nextftc.control.filters.Debouncer;
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

    private final NextMotor flywheel = new NextMotor("flywheel");
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
    private long lastTurretMoveMs = 0;
    private boolean atSpeed = false;

    /** True while a manual command owns the turret, suppressing pose tracking. */
    private boolean manualOverride = false;

    public Shooter(Supplier<Pose> poseSupplier) {
        this.poseSupplier = poseSupplier;
        turretRight.setDirection(NextMotor.Direction.REVERSE);
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
        targetRpm = 0;
        atSpeed = false;
        atSpeedDebouncer.calculate(false);
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

    /**
     * Whether the turret has had time to reach its commanded angle. Positional servos report
     * nothing, so this is elapsed time since the last large commanded move — if the turret is
     * bumped or stalls, this still reads true.
     */
    public boolean turretSettled() {
        return System.currentTimeMillis() - lastTurretMoveMs >= Constants.TURRET_SETTLE_MS;
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
            if (!manualOverride) {
                double bearing = bearingToGoal();
                applyTurret(Constants.clamp(bearing, Constants.TURRET_MIN_DEG, Constants.TURRET_MAX_DEG));
                applyHood(lookUp(Constants.DISTANCE_TO_HOOD, distanceToGoal(), commandedHoodPosition));
            }
            if (armed) {
                targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distanceToGoal(), targetRpm);
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

    public Command trimHood(double delta) {
        return instant(() -> {
            manualOverride = true;
            applyHood(Constants.clamp(commandedHoodPosition + delta,
                    Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
        });
    }

    /** Hands the hood and turret back to pose tracking after a manual override. */
    public Command resumeAutoAim() {
        return instant(() -> manualOverride = false);
    }

    // ---------------------------------------------------------------- hardware

    private void applyTurret(double degrees) {
        if (Math.abs(degrees - commandedTurretDeg) >= Constants.TURRET_MOVE_EPSILON_DEG) {
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
