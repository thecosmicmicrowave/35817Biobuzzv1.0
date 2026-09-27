package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Driving steps for autonomous routines.
 *
 * <p>An auto is one long {@code sequential(...)}, and Ivy's {@code Sequential} holds every
 * mechanism its steps use for the whole sequence. The shooter's and intake's default commands
 * therefore never run during auto, so each step here states explicitly what every mechanism does
 * instead of leaving gaps for a default to fill.
 */
public final class AutoCommands {

    private AutoCommands() {
    }

    /**
     * Follows a path while the turret tracks the goal, so the robot arrives already aimed. Ends
     * when the path ends.
     */
    public static Command driveAiming(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.shooter.autoAim());
    }

    /**
     * Follows a path while intaking and tracking the goal, then stops the rollers. The explicit
     * stop is required: the intake's default command does not run during auto, so without it the
     * rollers would keep spinning into the next step.
     */
    public static Command driveIntaking(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.intake.intake(),
                robot.shooter.autoAim())
                .then(robot.intake.stopIntake());
    }
}
