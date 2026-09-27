package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

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
 *
 * <p>Every step ends by holding the path's end pose. {@code follower.hold(...)} latches — the
 * follower keeps driving to that pose on every {@code update()} until the next path starts — so
 * the drivetrain stays commanded through whatever comes next, such as a two-second shot, instead
 * of coasting or being pushed off its spot.
 *
 * <p>The hold targets {@code path.endPose()}, deliberately not
 * {@code PedroCommands.hold(follower)}: that overload reads {@code follower.pose()} when the
 * command is <em>built</em>, which for an auto is at the start, so it would drive the robot back
 * to where the auto began.
 */
public final class AutoCommands {

    private AutoCommands() {
    }

    /**
     * Drives to a shooting spot ready to fire. Arms the flywheel before driving, so it spins up in
     * parallel with the drive rather than from a standstill on arrival, and tracks the goal with
     * the turret and hood while moving. Holds the end pose afterwards.
     *
     * <p>Without arming, {@code autoAim()} would hold the flywheel at zero for the whole drive —
     * it only follows the distance table once the shooter is armed — and every shot would start
     * with a full spin-up.
     */
    public static Command driveAiming(Robot robot, Path path) {
        return robot.shooter.spinUp()
                .then(deadline(
                        follow(robot.follower(), path),
                        robot.shooter.autoAim()))
                .then(hold(robot.follower(), path.endPose()));
    }

    /**
     * Follows a path while intaking and tracking the goal, then stops the rollers and holds the
     * end pose. The explicit stop is required: the intake's default command does not run during
     * auto, so without it the rollers would keep spinning into the next step.
     */
    public static Command driveIntaking(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.intake.intake(),
                robot.shooter.autoAim())
                .then(robot.intake.stopIntake())
                .then(hold(robot.follower(), path.endPose()));
    }
}
