package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.ivy.commands.Commands;

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
 * <h2>Turret: TURRET_CENTER_POSITION and TURRET_POSITION_PER_DEG</h2>
 * <ol>
 *   <li>Step the turret until it points straight forward. Press X to mark it — that position is
 *       {@code TURRET_CENTER_POSITION}.</li>
 *   <li>Step to a clearly different angle and press BACK to mark it.</li>
 *   <li>Measure that second angle off the robot with a protractor.</li>
 *   <li>Telemetry shows the position delta between the marks. Divide it by your measured degrees
 *       to get {@code TURRET_POSITION_PER_DEG}.</li>
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

    private double power = 0;
    private double turretPosition = Constants.TURRET_CENTER_POSITION;
    private double hoodPosition = Constants.HOOD_STOWED_POSITION;

    private Double centerMark = null;
    private Double referenceMark = null;

    public ShooterTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        driver = new CommandGamepad(gamepad1);
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
        // Scheduled once and never cancelled, so it owns the shooter for the whole OpMode.
        robot.shooter.manualControl(
                () -> power,
                () -> turretPosition,
                () -> hoodPosition
        ).schedule();
    }

    @Override
    public void periodic() {
        telemetry.addLine("flywheel: triggers step power, B stops");
        telemetry.addLine("turret:   dpad L/R fine, bumpers coarse");
        telemetry.addLine("hood:     dpad U/D fine, Y/A coarse");
        telemetry.addLine("marks:    X center, BACK reference");
        telemetry.addLine();

        double rpm = robot.shooter.currentRpm();
        telemetry.addLine("-- flywheel --");
        telemetry.addData("commanded power", "%.2f", power);
        telemetry.addData("applied throttle", "%.2f", robot.shooter.flywheelThrottle());
        telemetry.addData("measured RPM", "%.0f", rpm);
        telemetry.addData("current (A)", "%.2f", robot.shooter.flywheelCurrentAmps());
        if (Math.abs(rpm) > 1) {
            telemetry.addData("implied kV (power/RPM)", "%.8f", power / rpm);
        } else {
            telemetry.addData("implied kV (power/RPM)", "waiting for RPM");
        }
        telemetry.addData("ticks per rev in use", Constants.FLYWHEEL_TICKS_PER_REV);
        telemetry.addLine();

        telemetry.addLine("-- turret --");
        telemetry.addData("position", "%.4f", turretPosition);
        telemetry.addData("center mark", centerMark == null ? "unset" : format(centerMark));
        telemetry.addData("reference mark", referenceMark == null ? "unset" : format(referenceMark));
        if (centerMark != null && referenceMark != null) {
            telemetry.addData("position delta", "%.4f", referenceMark - centerMark);
            telemetry.addLine("TURRET_POSITION_PER_DEG = delta / measured degrees");
        }
        telemetry.addData("angle per current constants (deg)", "%.1f",
                Shooter.degreesForTurretPosition(turretPosition));
        telemetry.addData("center position in use", "%.4f", Constants.TURRET_CENTER_POSITION);
        telemetry.addData("position per deg in use", "%.6f", Constants.TURRET_POSITION_PER_DEG);
        telemetry.addLine();

        telemetry.addLine("-- hood --");
        telemetry.addData("position", "%.4f", hoodPosition);
        telemetry.addData("range in use", "%.2f .. %.2f",
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION);
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
