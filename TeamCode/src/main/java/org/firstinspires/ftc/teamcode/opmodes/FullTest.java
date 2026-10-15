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

/** Full test. Raw control of everything. */
@NextTeleop(name = "Full Test", group = "test")
public class FullTest extends NextOpMode {

    // ---------------------------------------------------------------- enable / disable
    // false = that part never commanded.

    private static final boolean ENABLE_FLYWHEEL = true;
    private static final boolean ENABLE_TURRET = true;
    private static final boolean ENABLE_HOOD = true;
    private static final boolean ENABLE_STAGE1 = true;
    private static final boolean ENABLE_STAGE2 = true;
    private static final boolean ENABLE_GATE = true;
    private static final boolean ENABLE_FLOWER = true;

    // ----------------------------------------------------------------

    private static final double POWER_STEP = 0.05;

    private final Robot robot;
    private final CommandGamepad driver;
    private double flywheelPower = 0;
    private double turretPosition = Constants.TURRET_CENTER_POSITION;
    private double hoodPosition = Constants.HOOD_STOWED_POSITION;
    private double stage1Power = 0;
    private double stage2Power = 0;
    private boolean gateOpen = false;
    private boolean flowerDeployed = false;

    /** Last loop time. Scale stick by time. */
    private long lastLoopNanos;

    public FullTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();
        // Drive on gamepad 2. Start pose default.
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        // Flywheel.
        driver.rightTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustFlywheel(POWER_STEP)));
        driver.leftTrigger().isOver(0.5).onTrue(Commands.instant(() -> adjustFlywheel(-POWER_STEP)));
        driver.b().onTrue(Commands.instant(() -> flywheelPower = 0));

        // Gate + flower toggles.
        driver.a().onTrue(Commands.instant(() -> gateOpen = !gateOpen));
        driver.x().onTrue(Commands.instant(() -> flowerDeployed = !flowerDeployed));
    }

    private double gateTarget() {
        return gateOpen ? Constants.GATE_OPEN_POSITION : Constants.GATE_CLOSED_POSITION;
    }

    private double flowerTarget() {
        return flowerDeployed ? Constants.FLOWER_DEPLOYED_POSITION : Constants.FLOWER_STOWED_POSITION;
    }

    private void adjustFlywheel(double delta) {
        flywheelPower = Constants.clamp(flywheelPower + delta, -1.0, 1.0);
    }

    private void drive() {
        // Gamepad 2 drive. SDK Y negative when up.
        robot.driveRobotCentric(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);
    }

    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();

        // Schedule once. Own both mechanisms whole OpMode. Disabled part = null = never commanded.
        robot.shooter.manualControl(
                ENABLE_FLYWHEEL ? () -> flywheelPower : null,
                ENABLE_TURRET ? () -> turretPosition : null,
                ENABLE_HOOD ? () -> hoodPosition : null
        ).schedule();

        robot.intake.manualControl(
                ENABLE_STAGE1 ? () -> stage1Power : null,
                ENABLE_STAGE2 ? () -> stage2Power : null,
                ENABLE_GATE ? this::gateTarget : null,
                ENABLE_FLOWER ? this::flowerTarget : null
        ).schedule();
    }

    @Override
    public void periodic() {
        readSticks();
        readStages();
        pushTelemetry();
        drive();
    }

    /** Left stick X = turret. Right stick Y = hood. */
    private void readSticks() {
        long now = System.nanoTime();
        // Cap. Slow loop no big jump.
        double dt = Math.min((now - lastLoopNanos) / 1e9, 0.1);
        lastLoopNanos = now;

        double turretStick = gamepad1.left_stick_x;
        if (Math.abs(turretStick) >= Constants.STICK_DEADZONE) {
            double degrees = turretStick * Constants.TURRET_STICK_DEG_PER_SEC * dt;
            turretPosition = Constants.clamp(
                    turretPosition + degrees * Constants.turretPositionPerDeg(), 0, 1);
        }

        // SDK Y negative when up.
        double hoodStick = -gamepad1.right_stick_y;
        if (Math.abs(hoodStick) >= Constants.STICK_DEADZONE) {
            hoodPosition += hoodStick * Constants.HOOD_STICK_POSITION_PER_SEC * dt;
        }
        hoodPosition = Constants.clamp(
                hoodPosition, Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION);
    }

    /** Dpad up/down = stage 1 in/out. Dpad right/left = stage 2 in/out. Hold to run. */
    private void readStages() {
        stage1Power = gamepad1.dpad_up ? Constants.FEED_POWER
                : gamepad1.dpad_down ? Constants.EJECT_POWER : 0;
        stage2Power = gamepad1.dpad_right ? Constants.FEED_POWER
                : gamepad1.dpad_left ? Constants.EJECT_POWER : 0;
    }

    private void pushTelemetry() {
        Telemetry.log("drive:    gamepad 2");
        Telemetry.log("flywheel: triggers step power, B stop");
        Telemetry.log("turret:   left stick X");
        Telemetry.log("hood:     right stick Y");
        Telemetry.log("stage 1:  dpad up / down");
        Telemetry.log("stage 2:  dpad right / left");
        Telemetry.log("gate:     A toggle");
        Telemetry.log("flower:   X toggle");
        Telemetry.log("");

        Telemetry.log("flywheel", !ENABLE_FLYWHEEL ? "DISABLED" : String.format(
                "power %.2f   %.0f RPM   %.2f A",
                flywheelPower, robot.shooter.currentRpm(), robot.shooter.flywheelCurrentAmps()));
        Telemetry.log("turret", !ENABLE_TURRET ? "DISABLED" : String.format("%.4f", turretPosition));
        Telemetry.log("hood", !ENABLE_HOOD ? "DISABLED" : String.format("%.4f", hoodPosition));
        Telemetry.log("stage 1", !ENABLE_STAGE1 ? "DISABLED" : String.format(
                "power %.2f   %.2f A", stage1Power, robot.intake.stage1CurrentAmps()));
        Telemetry.log("stage 2", !ENABLE_STAGE2 ? "DISABLED" : String.format(
                "power %.2f   %.2f A", stage2Power, robot.intake.stage2CurrentAmps()));
        Telemetry.log("gate", !ENABLE_GATE ? "DISABLED" : gateOpen ? "open" : "closed");
        Telemetry.log("flower", !ENABLE_FLOWER ? "DISABLED" : flowerDeployed ? "deployed" : "stowed");
    }

    @Override
    public void end() {
        flywheelPower = 0;
        if (ENABLE_FLYWHEEL) {
            // Cut power direct. Command already cancelled here.
            robot.shooter.coastFlywheel();
        }
    }
}
