package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Driver-controlled OpMode.
 *
 * <p>Driving goes through Pedro's own manual drive rather than NextFTC's {@code DriveCommands}.
 * The follower owns the drivetrain motors, so letting NextFTC drive them as well would mean two
 * writers fighting over the same hardware every loop.
 *
 * <p>Ordering matters. {@code follower.manual(...)} is called here in {@link #periodic()} and
 * {@code follower.update()} runs afterwards from {@code Robot.periodic()}, because NextFTC's
 * robot hook fires after the OpMode's own periodic. The follower must be updated every loop
 * while driving manually, which also keeps localization live for the shooter's auto-aim.
 */
@NextTeleop(name = "Main TeleOp")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;
    private final CommandGamepad driver;

    public MainTeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        // Send telemetry to Panels as well as the Driver Station. NextFTC dedupes backends and
        // Panels hands back the same wrapper every time, so re-registering per OpMode is safe.
        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.bindHardware();
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);
        robot.reset();

        driver = new CommandGamepad(gamepad1);
        bindControls();
    }

    private void bindControls() {
        driver.rightTrigger().isOver(0.5).onTrue(ShootCommands.shootOne(robot));
        driver.y().onTrue(ShootCommands.shootAll(robot));

        driver.leftBumper().whileTrue(robot.intake.intake());
        driver.rightBumper().whileTrue(robot.intake.eject());

        driver.a().onTrue(robot.intake.toggleFlower());

        // toggleOnTrue would not work here: spinUp() is instant, so it finishes on the frame
        // it starts and the second press would have nothing to cancel.
        driver.x().onTrue(Commands.conditional(
                robot.shooter::isArmed,
                robot.shooter.idleFlywheel(),
                robot.shooter.spinUp()));

        driver.dpadLeft().onTrue(robot.shooter.trimTurret(-Constants.TURRET_TRIM_STEP_DEG));
        driver.dpadRight().onTrue(robot.shooter.trimTurret(Constants.TURRET_TRIM_STEP_DEG));

        driver.dpadUp().onTrue(robot.shooter.trimHood(Constants.HOOD_TRIM_STEP));
        driver.dpadDown().onTrue(robot.shooter.trimHood(-Constants.HOOD_TRIM_STEP));

        // Clears any manual turret override and zeroes the trims.
        driver.back().onTrue(robot.shooter.resumeAutoAim());
    }

    @Override
    public void periodic() {
        drive();
        pushTelemetry();
    }

    private void drive() {
        if (robot.follower() == null) {
            return;
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robot.follower().pose().heading()
        );

        // driveOrHold rather than manual: when the sticks are released the follower holds its
        // current pose instead of coasting, so the robot resists being pushed while lining up
        // a shot. Swap for robot.follower().manual(powers) to let it coast instead.
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    private void pushTelemetry() {
        Telemetry.log("distance to goal", robot.shooter.distanceToGoal());
        Telemetry.log("turret (deg)", robot.shooter.commandedTurretDeg());
        Telemetry.log("hood", robot.shooter.commandedHoodPosition());
        Telemetry.log("flywheel target / actual", robot.shooter.targetRpm() + " / " + robot.shooter.currentRpm());
        Telemetry.log("at speed", robot.shooter.atSpeed());
        Telemetry.log("ready to shoot", robot.shooter.readyToShoot());
        Telemetry.log("gate open", robot.intake.isGateOpen());
        Telemetry.log("flower deployed", robot.intake.isFlowerDeployed());
    }
}
