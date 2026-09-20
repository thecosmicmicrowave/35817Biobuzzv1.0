package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Shot sequences, which span both mechanisms: the flywheel, turret, and hood belong to the
 * shooter while the gate and stage-2 feed belong to the intake. They live here rather than in
 * either mechanism so both stay independently understandable, and so autonomous runs the same
 * sequence teleop does.
 *
 * <p>Every wait is capped with {@code raceWith(waitMs(...))}. A flywheel that never reaches its
 * setpoint — belt slip, brownout, a bad tune — must not stall the scheduler for the rest of the
 * match.
 */
public final class ShootCommands {

    private ShootCommands() {
    }

    /** Spins up, waits for the shot to be ready, releases one artifact, returns to idle. */
    public static Command shootOne(Robot robot) {
        return robot.shooter.spinUp()
                .then(waitForShot(robot))
                .then(robot.intake.feed().raceWith(Commands.waitMs(Constants.FEED_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /**
     * Empties the robot into the goal. Both stages run for the whole volley and the flywheel
     * setpoint is held, so it recovers between artifacts instead of being re-armed each shot.
     */
    public static Command shootAll(Robot robot) {
        return robot.shooter.spinUp()
                .then(waitForShot(robot))
                .then(robot.intake.volley().raceWith(Commands.waitMs(Constants.VOLLEY_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /**
     * A shot from a known field position with no pose dependency, for autonomous and for pit
     * testing when localization is not trustworthy.
     */
    public static Command shootFrom(Robot robot, double rpm, double hoodPosition, double turretDeg) {
        return robot.shooter.spinUpTo(rpm)
                .then(robot.shooter.hoodTo(hoodPosition))
                .then(robot.shooter.turretTo(turretDeg))
                .then(waitForShot(robot))
                .then(robot.intake.feed().raceWith(Commands.waitMs(Constants.FEED_TIME_MS)))
                .then(robot.intake.stopFeed())
                .then(robot.shooter.idleFlywheel());
    }

    /** Waits for the flywheel and turret to be ready, giving up after the spin-up timeout. */
    private static Command waitForShot(Robot robot) {
        return Commands.waitUntil(robot.shooter::readyToShoot)
                .raceWith(Commands.waitMs(Constants.SPINUP_TIMEOUT_MS));
    }
}
