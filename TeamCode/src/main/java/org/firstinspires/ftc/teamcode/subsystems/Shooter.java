package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Goal;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.nextftc.control.filters.Debouncer;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/** Shooter. Flywheel, hood, turret. */
public class Shooter implements Mechanism {

    /** Flywheel motor. */
    private final NextMotor flywheel = new NextMotor(
            "flywheel", Units.Rotations.of(1.0 / Constants.FLYWHEEL_TICKS_PER_REV));
    // Not final. bindServos() replace each OpMode.
    private NextServo hood;
    private NextServo turretLeft;
    private NextServo turretRight;

    private final Supplier<Pose> poseSupplier;
    private final Supplier<Alliance> allianceSupplier;
    private final Supplier<Goal> goalSupplier;

    private final Debouncer atSpeedDebouncer =
            new Debouncer(Constants.AT_SPEED_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);

    /** Flywheel speed. Only truth. */
    private double targetRpm = 0;

    /** Armed: autoAim() track RPM from distance table. */
    private boolean armed = false;

    private double commandedTurretDeg = Constants.TURRET_CENTER_DEG;
    private double commandedHoodPosition = Constants.HOOD_STOWED_POSITION;
    private boolean atSpeed = false;

    /** Driver trim. Added on top of tracking. Stay across shots. */
    private double turretOffsetDeg = 0;
    private double hoodOffset = 0;

    /** Angle settle timer started for. Timer restart when turret drift past epsilon. */
    private double settledTargetDeg = Constants.TURRET_CENTER_DEG;
    private long lastTurretMoveMs = 0;

    /** Time current move need. Big move = more time. */
    private double settleDurationMs = 0;

    /** True: manualTurret() own turret. Tracking off. */
    private boolean manualOverride = false;

    /** Ticks-per-rev encoder scaling last built from. */
    private double appliedTicksPerRev = Constants.FLYWHEEL_TICKS_PER_REV;

    public Shooter(Supplier<Pose> poseSupplier, Supplier<Alliance> allianceSupplier,
                   Supplier<Goal> goalSupplier) {
        this.poseSupplier = poseSupplier;
        this.allianceSupplier = allianceSupplier;
        this.goalSupplier = goalSupplier;
        bindServos();
        applyTuning();
    }

    /** Make fresh hood + turret servos. Call every OpMode start. */
    public void bindServos() {
        hood = new NextServo("hood");
        turretLeft = new NextServo("turretLeft");
        turretRight = new NextServo("turretRight");
        configureTurretServo(turretLeft, Constants.TURRET_LEFT_REVERSED);
        configureTurretServo(turretRight, Constants.TURRET_RIGHT_REVERSED);
    }

    /** Set turret servo pulse range + direction. Apply when hardware first used. */
    private static void configureTurretServo(NextServo servo, boolean reversed) {
        servo.setPwmRange(Constants.TURRET_SERVO_PWM_MIN_US, Constants.TURRET_SERVO_PWM_MAX_US);
        servo.setDirection(reversed ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
    }

    /** Copy tunables into motor gains, encoder scaling, debouncer. */
    private void applyTuning() {
        MotorVelocityConstants velocity = flywheel.getVelocityConstants();
        velocity.setKP(Constants.FLYWHEEL_KP);
        velocity.setKI(Constants.FLYWHEEL_KI);
        velocity.setKD(Constants.FLYWHEEL_KD);
        velocity.setKS(Constants.FLYWHEEL_KS);
        velocity.setKV(Constants.FLYWHEEL_KV);
        velocity.setKA(Constants.FLYWHEEL_KA);

        // Rebuild only on change. No new Angle every loop.
        if (Constants.FLYWHEEL_TICKS_PER_REV != appliedTicksPerRev) {
            flywheel.setAnglePerCount(Units.Rotations.of(1.0 / Constants.FLYWHEEL_TICKS_PER_REV));
            appliedTicksPerRev = Constants.FLYWHEEL_TICKS_PER_REV;
        }

        atSpeedDebouncer.setDebounceTimeSeconds(Constants.AT_SPEED_DEBOUNCE_SECONDS);
    }

    // ---------------------------------------------------------------- lifecycle

    /** Known state. Clear leftover state from last OpMode. */
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
        applyTuning();
        atSpeed = atSpeedDebouncer.calculate(withinRpmTolerance());
    }

    @Override
    public Command getDefaultCommand() {
        return autoAim();
    }

    // ---------------------------------------------------------------- state

    public double distanceToGoal() {
        Pose pose = poseSupplier.get();
        Alliance alliance = allianceSupplier.get();
        Goal goal = goalSupplier.get();
        double dx = Constants.goalX(alliance, goal) - pose.x();
        double dy = Constants.goalY(alliance, goal) - pose.y();
        return Math.hypot(dx, dy);
    }

    /** Goal bearing, degrees from robot forward. */
    public double bearingToGoal() {
        Pose pose = poseSupplier.get();
        Alliance alliance = allianceSupplier.get();
        Goal goal = goalSupplier.get();
        double dx = Constants.goalX(alliance, goal) - pose.x();
        double dy = Constants.goalY(alliance, goal) - pose.y();
        double fieldBearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        return Constants.normalizeDeg(fieldBearingDeg - Math.toDegrees(pose.heading()));
    }

    public boolean inTurretRange(double degrees) {
        return degrees >= Constants.turretMinDeg() && degrees <= Constants.turretMaxDeg();
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

    /** Motor power now. */
    public double flywheelThrottle() {
        return flywheel.getThrottle();
    }

    /** Cut flywheel now, no command. For OpMode end. */
    public void coastFlywheel() {
        targetRpm = 0;
        armed = false;
        flywheel.setThrottle(0);
    }

    /** Flywheel amps. */
    public double flywheelCurrentAmps() {
        return flywheel.getCurrent().into(Units.Amperes);
    }

    /** Turret had time to arrive. Time guess only, servo no report. */
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

    /** Default command. Aim at goal, refresh flywheel. Other commands interrupt, it come back. */
    public Command autoAim() {
        return infinite(this::aimAtGoal);
    }

    /** Aim turret + hood at goal. Done when turret arrive. */
    public Command aim() {
        // Aim in start too, so settle timer running before first done check.
        return infinite(this::aimAtGoal)
                .setStart(this::aimAtGoal)
                .setDone(this::turretSettled);
    }

    /** One aim step. Turret + hood from pose, flywheel refreshed. */
    private void aimAtGoal() {
        double distance = distanceToGoal();

        // Manual override take turret only. Hood keep tracking.
        if (!manualOverride) {
            applyTurret(Constants.clamp(bearingToGoal() + turretOffsetDeg,
                    Constants.turretMinDeg(), Constants.turretMaxDeg()));
        }
        applyHood(Constants.clamp(
                lookUp(Constants.DISTANCE_TO_HOOD, distance, commandedHoodPosition) + hoodOffset,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
        if (armed) {
            targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distance, targetRpm);
        }
        applyFlywheel();
    }

    /** Arm, no command. Flywheel hold table RPM from next aim loop. */
    public void arm() {
        armed = true;
    }

    /** Arm. Spin to table RPM for current distance. */
    public Command spinUp() {
        return instant(() -> {
            armed = true;
            targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distanceToGoal(), targetRpm);
            applyFlywheel();
        });
    }

    /** Fixed RPM, ignore pose. */
    public Command spinUpTo(double rpm) {
        return instant(() -> {
            armed = false;
            targetRpm = rpm;
            applyFlywheel();
        });
    }

    /** Drop to idle RPM, not zero. Next spin-up faster. */
    public Command idleFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = Constants.IDLE_RPM;
            applyFlywheel();
        });
    }

    /** Do nothing, own shooter. autoAim() off, nothing move. */
    public Command standby() {
        return infinite(() -> {
        });
    }

    public Command stopFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = 0;
            applyFlywheel();
        });
    }

    /** Raw control for testing. Flywheel power, turret + hood servo position. Null = leave alone. */
    public Command manualControl(
            DoubleSupplier power, DoubleSupplier turretPosition, DoubleSupplier hoodPosition) {
        return infinite(() -> {
            if (power != null) {
                flywheel.setThrottle(Constants.clamp(power.getAsDouble(), -1.0, 1.0));
            }

            if (turretPosition != null) {
                double turret = Constants.clamp(turretPosition.getAsDouble(), 0, 1);
                // Directions set in bindServos(). Both get same value.
                turretLeft.setPosition(turret);
                turretRight.setPosition(turret);
            }

            if (hoodPosition != null) {
                hood.setPosition(Constants.clamp(hoodPosition.getAsDouble(), 0, 1));
            }
        });
    }

    /** Turret angle current constants think a raw position is. */
    public static double degreesForTurretPosition(double position) {
        return Constants.TURRET_CENTER_DEG
                + (position - Constants.TURRET_CENTER_POSITION) / Constants.turretPositionPerDeg();
    }

    /** Driver nudge turret. Interrupt autoAim(), it come back on release. */
    public Command manualTurret(DoubleSupplier input) {
        return infinite(() -> {
            manualOverride = true;
            double delta = input.getAsDouble() * Constants.TURRET_TRIM_DEG_PER_LOOP;
            applyTurret(Constants.clamp(commandedTurretDeg + delta,
                    Constants.turretMinDeg(), Constants.turretMaxDeg()));
            applyFlywheel();
        }).setEnd(endCondition -> manualOverride = false);
    }

    public Command turretTo(double degrees) {
        return instant(() -> applyTurret(Constants.clamp(degrees,
                Constants.turretMinDeg(), Constants.turretMaxDeg())));
    }

    public Command centerTurret() {
        return turretTo(Constants.TURRET_CENTER_DEG);
    }

    /** Set hood position. */
    public Command hoodTo(double position) {
        return instant(() -> applyHood(Constants.clamp(position,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION)));
    }

    /** Shift hood trim. Tracking keep going. */
    public Command trimHood(double delta) {
        return instant(() -> hoodOffset += delta);
    }

    /** Shift turret trim. */
    public Command trimTurret(double deltaDeg) {
        return instant(() -> turretOffsetDeg += deltaDeg);
    }

    /** Clear override + trims. Back to pure tracking. */
    public Command resumeAutoAim() {
        return instant(() -> {
            manualOverride = false;
            turretOffsetDeg = 0;
            hoodOffset = 0;
        });
    }

    // ---------------------------------------------------------------- hardware

    private void applyTurret(double degrees) {
        // Compare to angle timer started for, not last call. autoAim() move tiny bit each loop.
        double moveDeg = Math.abs(degrees - settledTargetDeg);
        if (moveDeg >= Constants.TURRET_MOVE_EPSILON_DEG) {
            settleDurationMs = Constants.TURRET_SETTLE_BASE_MS
                    + moveDeg * Constants.TURRET_SETTLE_MS_PER_DEG;
            settledTargetDeg = degrees;
            lastTurretMoveMs = System.currentTimeMillis();
        }
        commandedTurretDeg = degrees;

        double position = Constants.TURRET_CENTER_POSITION
                + (degrees - Constants.TURRET_CENTER_DEG) * Constants.turretPositionPerDeg();
        position = Constants.clamp(position, 0, 1);

        // Directions set in bindServos(). Both get same position.
        turretLeft.setPosition(position);
        turretRight.setPosition(position);
    }

    /** Send targetRpm to motor. Safe to call anytime. */
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

    /** Read table. Key clamped to table range. Empty table = fallback. */
    private static double lookUp(
            dev.nextftc.control.util.InterpolatingMap<Double> map, double key, double fallback) {
        if (map.isEmpty()) {
            return fallback;
        }
        // Table throw on key outside range. Clamp to nearest entry.
        double clamped = Constants.clamp(key, map.firstKey(), map.lastKey());
        Double value = map.get(clamped);
        return value == null ? fallback : value;
    }
}
