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
 * Turret and hood calibration.
 *
 * <p>Drives both on raw servo positions, bypassing the degree mapping in {@code turretTo()} —
 * that mapping is what this OpMode establishes, so it cannot be used to find itself.
 *
 * <p>The flywheel is cut at start and never commanded, and the held calibration command keeps
 * {@code autoAim()} suspended, so nothing moves except what the dpad and bumpers move.
 *
 * <h2>Finding TURRET_CENTER_POSITION and TURRET_POSITION_PER_DEG</h2>
 * <ol>
 *   <li>Step the turret until it points straight forward. Press X to mark it — that position is
 *       {@code TURRET_CENTER_POSITION}.</li>
 *   <li>Step the turret to a clearly different angle and press B to mark it.</li>
 *   <li>Measure that second angle off the robot with a protractor.</li>
 *   <li>Telemetry shows the position delta between the marks. Divide it by your measured
 *       degrees to get {@code TURRET_POSITION_PER_DEG}.</li>
 * </ol>
 *
 * <p>For the travel limits, step carefully outward until the turret reaches its mechanical stop,
 * back off, and read the inferred degrees — those become {@code TURRET_MIN_DEG} and
 * {@code TURRET_MAX_DEG}. Set them conservatively; they are the only thing preventing a
 * hardstop crash once auto-aim is live.
 *
 * <p><b>Step in fine increments and stop the moment a servo buzzes or the mechanism binds.</b>
 * A servo held against a hardstop will burn out.
 */
@NextTeleop(name = "Servo Calibration", group = "test")
public class ServoCalibrationTest extends NextOpMode {

    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.02;

    private final Robot robot;
    private final CommandGamepad driver;

    private double turretPosition = Constants.TURRET_CENTER_POSITION;
    private double hoodPosition = Constants.HOOD_STOWED_POSITION;

    private Double centerMark = null;
    private Double referenceMark = null;

    public ServoCalibrationTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // No mechanism requirements, so these never preempt the held calibration command.
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustTurret(-FINE_STEP)));
        driver.dpadRight().onTrue(Commands.instant(() -> adjustTurret(FINE_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> adjustTurret(-COARSE_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> adjustTurret(COARSE_STEP)));

        driver.dpadDown().onTrue(Commands.instant(() -> adjustHood(-FINE_STEP)));
        driver.dpadUp().onTrue(Commands.instant(() -> adjustHood(FINE_STEP)));
        driver.a().onTrue(Commands.instant(() -> adjustHood(-COARSE_STEP)));
        driver.y().onTrue(Commands.instant(() -> adjustHood(COARSE_STEP)));

        driver.x().onTrue(Commands.instant(() -> centerMark = turretPosition));
        driver.b().onTrue(Commands.instant(() -> referenceMark = turretPosition));
    }

    private void adjustTurret(double delta) {
        turretPosition = Constants.clamp(turretPosition + delta, 0, 1);
    }

    private void adjustHood(double delta) {
        hoodPosition = Constants.clamp(hoodPosition + delta, 0, 1);
    }

    @Override
    public void start() {
        robot.shooter.coastFlywheel();
        // Scheduled once and never cancelled, so it owns the shooter for the whole OpMode.
        robot.shooter.calibrateServos(() -> turretPosition, () -> hoodPosition).schedule();
    }

    @Override
    public void periodic() {
        telemetry.addLine("turret: dpad L/R fine, bumpers coarse");
        telemetry.addLine("hood:   dpad U/D fine, Y/A coarse");
        telemetry.addLine("X: mark center   B: mark reference");
        telemetry.addLine();

        telemetry.addData("turret position", "%.4f", turretPosition);
        telemetry.addData("hood position", "%.4f", hoodPosition);
        telemetry.addLine();

        telemetry.addData("center mark", centerMark == null ? "unset" : format(centerMark));
        telemetry.addData("reference mark", referenceMark == null ? "unset" : format(referenceMark));

        if (centerMark != null && referenceMark != null) {
            double delta = referenceMark - centerMark;
            telemetry.addData("position delta", "%.4f", delta);
            telemetry.addLine("TURRET_POSITION_PER_DEG = delta / measured degrees");
        }

        telemetry.addLine();
        telemetry.addLine("-- what the current constants infer --");
        telemetry.addData("turret angle (deg)", "%.1f",
                Shooter.degreesForTurretPosition(turretPosition));
        telemetry.addData("center position in use", "%.4f", Constants.TURRET_CENTER_POSITION);
        telemetry.addData("position per deg in use", "%.6f", Constants.TURRET_POSITION_PER_DEG);
    }

    private static String format(double value) {
        return String.format("%.4f", value);
    }
}
