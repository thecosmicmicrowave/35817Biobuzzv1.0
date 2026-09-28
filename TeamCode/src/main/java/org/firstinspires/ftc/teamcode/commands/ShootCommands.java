package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

/** Shot sequences. */
public final class ShootCommands {

    private ShootCommands() {
    }

    /** Spin up, wait ready, shoot one, idle. */
    public static Command shootOne(Robot robot) {
        return robot.shooter.spinUp()
                .then(waitForShot(robot))
                .then(robot.intake.feed().raceWith(Commands.waitMs(Constants.FEED_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /** Spin up, wait ready, shoot all, idle. Flywheel held between artifacts. */
    public static Command shootAll(Robot robot) {
        return robot.shooter.spinUp()
                .then(waitForShot(robot))
                .then(robot.intake.volley().raceWith(Commands.waitMs(Constants.VOLLEY_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /** Shoot with fixed settings, no pose. Auto or pit test. */
    public static Command shootFrom(Robot robot, double rpm, double hoodPosition, double turretDeg) {
        return robot.shooter.spinUpTo(rpm)
                .then(robot.shooter.hoodTo(hoodPosition))
                .then(robot.shooter.turretTo(turretDeg))
                .then(waitForShot(robot))
                .then(robot.intake.feed().raceWith(Commands.waitMs(Constants.FEED_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /** Wait flywheel + turret ready. Give up after timeout. */
    private static Command waitForShot(Robot robot) {
        return Commands.waitUntil(robot.shooter::readyToShoot)
                .raceWith(Commands.waitMs(Constants.SPINUP_TIMEOUT_MS));
    }
}
