package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Main TeleOp")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;
    private final CommandGamepad driver;

    public MainTeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

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
    public void start() {
        DriveCommands.mecanumDriveFieldCentric(
                robot.frontLeft,
                robot.frontRight,
                robot.backLeft,
                robot.backRight,
                gamepad1,
                () -> robot.pose().heading()
        ).schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("distance to goal", robot.shooter.distanceToGoal());
        telemetry.addData("turret (deg)", robot.shooter.commandedTurretDeg());
        telemetry.addData("hood", robot.shooter.commandedHoodPosition());
        telemetry.addData("flywheel target / actual",
                robot.shooter.targetRpm() + " / " + robot.shooter.currentRpm());
        telemetry.addData("at speed", robot.shooter.atSpeed());
        telemetry.addData("ready to shoot", robot.shooter.readyToShoot());
        telemetry.addData("gate open", robot.intake.isGateOpen());
        telemetry.addData("flower deployed", robot.intake.isFlowerDeployed());
    }
}
