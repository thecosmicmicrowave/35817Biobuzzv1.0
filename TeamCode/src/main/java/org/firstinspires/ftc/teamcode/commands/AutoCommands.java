package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.globals.Robot;

/**
 * Auto drive steps.
 *
 * <p>Auto = one big sequential. Mechanism defaults no run in auto. So each step say what every
 * mechanism do.
 *
 * <p>Each step end with hold on path end pose. Hold stay until next path. Robot no coast.
 * Use path.endPose(), NOT PedroCommands.hold(follower): that one grab pose at build time =
 * auto start pose.
 */
public final class AutoCommands {

    private AutoCommands() {
    }

    /**
     * Drive to shoot spot. Arm flywheel first so spin while driving. Turret + hood aim while
     * moving. Hold end pose.
     */
    public static Command driveAiming(Robot robot, Path path) {
        return robot.shooter.spinUp()
                .then(deadline(
                        follow(robot.follower(), path),
                        robot.shooter.autoAim()))
                .then(hold(robot.follower(), path.endPose()));
    }

    /**
     * Drive + intake + aim. Then stop rollers (default no run in auto), hold end pose.
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
