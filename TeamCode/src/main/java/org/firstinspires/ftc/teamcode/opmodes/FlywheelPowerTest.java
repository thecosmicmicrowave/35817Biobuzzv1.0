package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Open-loop flywheel bring-up.
 *
 * <p>Runs the flywheel on raw motor power, bypassing the velocity controller entirely, to answer
 * the first three questions before any gain is trusted: does the motor spin, does it spin the
 * right way, and does the encoder read a sane speed.
 *
 * <p>Holds the shooter mechanism for the whole OpMode, which keeps {@code autoAim()} suspended
 * so the turret and hood are never commanded. That matters during bring-up: the turret
 * calibration constants are placeholders, and a wrong {@code TURRET_POSITION_PER_DEG} would
 * drive the turret into its hardstop the moment a normal OpMode started.
 *
 * <p>The implied kV reading is the point of the telemetry. Hold a steady power, wait for the RPM
 * to settle, and read it — that value goes straight into {@code Constants.FLYWHEEL_KV}. Take it
 * at a power near your actual shooting speed rather than at full throttle.
 */
@NextTeleop(name = "Flywheel Power Test", group = "test")
public class FlywheelPowerTest extends NextOpMode {

    private static final double POWER_STEP = 0.05;

    private final Robot robot;
    private final CommandGamepad driver;

    private double commandedPower = 0;

    public FlywheelPowerTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // These require no mechanism, so they do not preempt the held flywheelPower command.
        driver.dpadUp().onTrue(Commands.instant(() -> adjustPower(POWER_STEP)));
        driver.dpadDown().onTrue(Commands.instant(() -> adjustPower(-POWER_STEP)));
        driver.b().onTrue(Commands.instant(() -> commandedPower = 0));
        driver.y().onTrue(Commands.instant(() -> commandedPower = 1.0));
    }

    private void adjustPower(double delta) {
        commandedPower = Constants.clamp(commandedPower + delta, -1.0, 1.0);
    }

    @Override
    public void start() {
        // Scheduled once and never cancelled, so it owns the shooter for the whole OpMode.
        robot.shooter.flywheelPower(() -> commandedPower).schedule();
    }

    @Override
    public void periodic() {
        double rpm = robot.shooter.currentRpm();

        telemetry.addLine("dpad up/down: step power | B: stop | Y: full");
        telemetry.addLine();
        telemetry.addData("commanded power", "%.2f", commandedPower);
        telemetry.addData("applied throttle", "%.2f", robot.shooter.flywheelThrottle());
        telemetry.addData("measured RPM", "%.0f", rpm);
        telemetry.addData("current (A)", "%.2f", robot.shooter.flywheelCurrentAmps());

        // power / rpm at a settled speed is the feedforward gain.
        if (Math.abs(rpm) > 1) {
            telemetry.addData("implied kV (power/RPM)", "%.8f", commandedPower / rpm);
        } else {
            telemetry.addData("implied kV (power/RPM)", "waiting for RPM");
        }

        telemetry.addLine();
        telemetry.addData("ticks per rev in use", Constants.FLYWHEEL_TICKS_PER_REV);
    }

    @Override
    public void end() {
        // Cut power directly. Zeroing the field alone would do nothing, since the command
        // holding the flywheel is already cancelled by the time this runs.
        commandedPower = 0;
        robot.shooter.coastFlywheel();
    }
}
