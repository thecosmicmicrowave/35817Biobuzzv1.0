package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Gate test. Tune gate open + closed positions. */
@NextTeleop(name = "Gate Test", group = "test")
public class GateTest extends NextOpMode {

    private static final double FINE_STEP = 0.005;
    private static final double COARSE_STEP = 0.02;

    private final Robot robot;
    private final CommandGamepad driver;

    private double position = Constants.GATE_CLOSED_POSITION;

    public GateTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // Move gate.
        driver.dpadUp().onTrue(Commands.instant(() -> adjust(FINE_STEP)));
        driver.dpadDown().onTrue(Commands.instant(() -> adjust(-FINE_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> adjust(COARSE_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> adjust(-COARSE_STEP)));

        // Jump to saved open / closed.
        driver.a().onTrue(Commands.instant(() -> position = Constants.GATE_OPEN_POSITION));
        driver.b().onTrue(Commands.instant(() -> position = Constants.GATE_CLOSED_POSITION));

        // Save current as open / closed. Live only, copy into Constants.java.
        driver.x().onTrue(Commands.instant(() -> Constants.GATE_OPEN_POSITION = position));
        driver.y().onTrue(Commands.instant(() -> Constants.GATE_CLOSED_POSITION = position));
    }

    private void adjust(double delta) {
        position = Constants.clamp(position + delta, 0, 1);
    }

    @Override
    public void start() {
        // Schedule once. Own intake whole OpMode.
        robot.intake.gateTo(() -> position).schedule();
    }

    @Override
    public void periodic() {
        Telemetry.log("move:  dpad U/D fine, bumpers coarse");
        Telemetry.log("jump:  A = open, B = closed");
        Telemetry.log("save:  X = open, Y = closed");
        Telemetry.log("");
        Telemetry.log("gate position", String.format("%.3f", position));
        Telemetry.log("GATE_OPEN_POSITION", String.format("%.3f", Constants.GATE_OPEN_POSITION));
        Telemetry.log("GATE_CLOSED_POSITION", String.format("%.3f", Constants.GATE_CLOSED_POSITION));
    }
}
