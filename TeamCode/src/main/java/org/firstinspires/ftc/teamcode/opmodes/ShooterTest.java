package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Shooter bring-up and calibration: flywheel, turret, and hood in one OpMode.
 *
 * <p>Everything runs open loop. The flywheel takes raw throttle instead of a velocity setpoint,
 * so the untuned velocity loop cannot interfere. The turret and hood take raw servo positions,
 * bypassing the degree mapping in {@code turretTo()} — that mapping is what this OpMode
 * establishes, so it cannot be used to find itself.
 *
 * <p>A single held command owns the shooter for the whole run, which keeps {@code autoAim()}
 * suspended. Without that, {@code autoAim()} being the shooter's default command would swing the
 * turret to its computed bearing the moment the OpMode started, using calibration constants that
 * are still placeholders. Nothing moves here except what the controls move, and the flywheel
 * always starts at zero.
 *
 * <h2>Flywheel: FLYWHEEL_TICKS_PER_REV and FLYWHEEL_KV</h2>
 * <p>Step the power up, let the RPM settle, and read the implied kV — that value goes straight
 * into {@code Constants.FLYWHEEL_KV}. Take it near your intended shooting speed rather than at
 * full throttle, since the relationship is not perfectly linear at the extremes. If the RPM is
 * wildly implausible, {@code FLYWHEEL_TICKS_PER_REV} is wrong for this motor. If it reads zero
 * while the flywheel is clearly spinning, the encoder is not wired.
 *
 * <h2>Turret: TURRET_CENTER_POSITION, then verifying the gearing</h2>
 * <p>{@code TURRET_POSITION_PER_DEG} is derived from the gear train and the servo travel, so it
 * is not measured here — it is checked.
 * <ol>
 *   <li>Step the turret until it points straight forward. Press X to mark it, and set
 *       {@code TURRET_CENTER_POSITION} to that value in Panels.</li>
 *   <li>Step to a clearly different angle and press BACK to mark it.</li>
 *   <li>Measure that angle off the robot with a protractor and compare it with the predicted
 *       angle between marks on telemetry.</li>
 *   <li>If they disagree, adjust {@code TURRET_SERVO_TRAVEL_DEG} in Panels until they match. A
 *       value near 270 means the servos were never reprogrammed to 320; any other mismatch means
 *       the PWM range is not reaching the servo's full travel.</li>
 * </ol>
 *
 * <p>For the travel limits, step carefully outward to the mechanical stop, back off, and read the
 * inferred degrees — those become {@code TURRET_MIN_DEG} and {@code TURRET_MAX_DEG}. Set them
 * conservatively; they are the only thing preventing a hardstop crash once auto-aim is live.
 *
 * <p><b>Step servos in fine increments and stop the moment one buzzes or the mechanism binds.</b>
 * A servo held against a hardstop will burn out, and with two ganged turret servos you cannot
 * hear which one is fighting.
 */
@NextTeleop(name = "Shooter Test", group = "test")
public class ShooterTest extends NextOpMode {

    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.02;
    private static final double POWER_STEP = 0.05;

    private final Robot robot;
    private final CommandGamepad driver;
    private final CommandGamepad operator;

    private double power = 0;

    /** Time of the previous loop, for scaling stick input by elapsed time. */
    private long lastLoopNanos;
    private double turretPosition = Constants.TURRET_CENTER_POSITION;
    private double hoodPosition = Constants.HOOD_STOWED_POSITION;

    private Double centerMark = null;
    private Double referenceMark = null;

    public ShooterTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        // Send telemetry to Panels as well as the Driver Station. NextFTC dedupes backends and
        // Panels hands back the same wrapper every time, so re-registering per OpMode is safe.
        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driver = new CommandGamepad(gamepad1);
        operator = new CommandGamepad(gamepad2);
        bindControls();
    }

    private void bindControls() {
        // None of these require a mechanism, so they never preempt the held manualControl command.

        // Flywheel.
        driver.rightTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustPower(POWER_STEP)));
        driver.leftTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustPower(-POWER_STEP)));
        driver.b().onTrue(Commands.instant(() -> power = 0));

        // Turret.
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustTurret(-FINE_STEP)));
        driver.dpadRight().onTrue(Commands.instant(() -> adjustTurret(FINE_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> adjustTurret(-COARSE_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> adjustTurret(COARSE_STEP)));

        // Hood.
        driver.dpadDown().onTrue(Commands.instant(() -> adjustHood(-FINE_STEP)));
        driver.dpadUp().onTrue(Commands.instant(() -> adjustHood(FINE_STEP)));
        driver.a().onTrue(Commands.instant(() -> adjustHood(-COARSE_STEP)));
        driver.y().onTrue(Commands.instant(() -> adjustHood(COARSE_STEP)));

        // Calibration marks.
        driver.x().onTrue(Commands.instant(() -> centerMark = turretPosition));
        driver.back().onTrue(Commands.instant(() -> referenceMark = turretPosition));

        // Feeding, on gamepad2 since gamepad1 is full. Without these nothing ever moves an
        // artifact into the flywheel, so a spinning flywheel alone cannot show whether it shoots.
        // These use the intake mechanism, which the held manualControl command does not own, so
        // they run alongside it. The gate is closed again on release: the intake's default command
        // only stops the motors and deliberately leaves the servos where they are.
        operator.leftBumper().whileTrue(robot.intake.intake());
        operator.rightBumper().whileTrue(robot.intake.volley())
                .onFalse(robot.intake.closeGate());
        operator.b().whileTrue(robot.intake.eject());
    }

    /**
     * Left stick X moves the turret continuously. Scaled by elapsed time so the turret turns at
     * TURRET_STICK_DEG_PER_SEC regardless of loop rate, and converted from turret degrees to raw
     * servo position through the gear ratio so the speed means the same thing at the turret.
     */
    private void driveTurretFromStick() {
        long now = System.nanoTime();
        // Capped so a stalled loop cannot turn into one large jump.
        double dt = Math.min((now - lastLoopNanos) / 1e9, 0.1);
        lastLoopNanos = now;

        double stick = gamepad1.left_stick_x;
        if (Math.abs(stick) < Constants.STICK_DEADZONE) {
            return;
        }
        double degrees = stick * Constants.TURRET_STICK_DEG_PER_SEC * dt;
        adjustTurret(degrees * Constants.turretPositionPerDeg());
    }

    private void adjustPower(double delta) {
        power = Constants.clamp(power + delta, -1.0, 1.0);
    }

    private void adjustTurret(double delta) {
        turretPosition = Constants.clamp(turretPosition + delta, 0, 1);
    }

    private void adjustHood(double delta) {
        hoodPosition = Constants.clamp(hoodPosition + delta, 0, 1);
    }

    @Override
    public void start() {
        // Start from a known gate position rather than wherever the servo was left.
        robot.intake.closeGate().schedule();
        lastLoopNanos = System.nanoTime();
        // Scheduled once and never cancelled, so it owns the shooter for the whole OpMode.
        robot.shooter.manualControl(
                () -> power,
                () -> turretPosition,
                () -> hoodPosition
        ).schedule();
    }

    @Override
    public void periodic() {
        driveTurretFromStick();

        Telemetry.log("flywheel: triggers step power, B stops");
        Telemetry.log("turret:   left stick X, dpad L/R fine, bumpers coarse");
        Telemetry.log("hood:     dpad U/D fine, Y/A coarse");
        Telemetry.log("marks:    X center, BACK reference");
        Telemetry.log("gamepad2: LB intake, RB hold to feed/shoot, B eject");
        Telemetry.log("");

        double rpm = robot.shooter.currentRpm();
        Telemetry.log("-- flywheel --");
        Telemetry.log("commanded power", String.format("%.2f", power));
        Telemetry.log("applied throttle", String.format("%.2f", robot.shooter.flywheelThrottle()));
        Telemetry.log("measured RPM", String.format("%.0f", rpm));
        Telemetry.log("current (A)", String.format("%.2f", robot.shooter.flywheelCurrentAmps()));
        if (Math.abs(rpm) > 1) {
            Telemetry.log("implied kV (power/RPM)", String.format("%.8f", power / rpm));
        } else {
            Telemetry.log("implied kV (power/RPM)", "waiting for RPM");
        }
        Telemetry.log("ticks per rev in use", Constants.FLYWHEEL_TICKS_PER_REV);
        Telemetry.log("");

        Telemetry.log("-- turret --");
        Telemetry.log("turret position", String.format("%.4f", turretPosition));
        Telemetry.log("center mark", centerMark == null ? "unset" : format(centerMark));
        Telemetry.log("reference mark", referenceMark == null ? "unset" : format(referenceMark));
        if (centerMark != null && referenceMark != null) {
            double delta = referenceMark - centerMark;
            Telemetry.log("position delta", String.format("%.4f", delta));
            Telemetry.log("predicted angle between marks (deg)",
                    String.format("%.1f", delta / Constants.turretPositionPerDeg()));
            Telemetry.log("compare with protractor; if off, tune TURRET_SERVO_TRAVEL_DEG");
        }
        Telemetry.log("angle per current constants (deg)", String.format("%.1f", Shooter.degreesForTurretPosition(turretPosition)));
        Telemetry.log("center position in use", String.format("%.4f", Constants.TURRET_CENTER_POSITION));
        Telemetry.log("position per deg in use", String.format("%.6f", Constants.turretPositionPerDeg()));
        Telemetry.log("");

        Telemetry.log("-- hood --");
        Telemetry.log("hood position", String.format("%.4f", hoodPosition));
        Telemetry.log("range in use", String.format("%.2f .. %.2f", Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
    }

    @Override
    public void end() {
        // Cut power directly. Zeroing the field alone would do nothing, since the command
        // holding the flywheel is already cancelled by the time this runs.
        power = 0;
        robot.shooter.coastFlywheel();
    }

    private static String format(double value) {
        return String.format("%.4f", value);
    }
}
