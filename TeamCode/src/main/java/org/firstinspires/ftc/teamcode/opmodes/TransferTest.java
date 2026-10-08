package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

/** Transfer test. Raw power stage 1 + stage 2. No gate. */
@NextTeleop(name = "Transfer Test", group = "test")
public class TransferTest extends NextOpMode {

    private final Robot robot;

    private double stage1Power = 0;
    private double stage2Power = 0;

    public TransferTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();
    }

    @Override
    public void start() {
        // Schedule once. Own intake whole OpMode.
        robot.intake.rollersTo(() -> stage1Power, () -> stage2Power).schedule();
        // Hold shooter. No auto-aim, turret stay still.
        robot.shooter.standby().schedule();
    }

    @Override
    public void periodic() {
        // Stick up = forward. SDK Y negative when up.
        stage1Power = stick(-gamepad1.left_stick_y);
        stage2Power = stick(-gamepad1.right_stick_y);

        Telemetry.log("stage 1: left stick Y");
        Telemetry.log("stage 2: right stick Y");
        Telemetry.log("");
        Telemetry.log("stage 1 power", String.format("%.2f", stage1Power));
        Telemetry.log("stage 1 current (A)", String.format("%.2f", robot.intake.stage1CurrentAmps()));
        Telemetry.log("stage 2 power", String.format("%.2f", stage2Power));
        Telemetry.log("stage 2 current (A)", String.format("%.2f", robot.intake.stage2CurrentAmps()));
    }

    /** Stick with deadzone. */
    private static double stick(double value) {
        return Math.abs(value) < Constants.STICK_DEADZONE ? 0 : value;
    }
}
