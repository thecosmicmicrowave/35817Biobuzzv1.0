package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.globals.Robot;

/** Auto drive steps. */
public final class AutoCommands {

    private AutoCommands() {
    }

    /** Drive to shoot spot. Spin flywheel + aim while driving. Hold end pose. */
    public static Command driveAiming(Robot robot, Path path) {
        return robot.shooter.spinUp()
                .then(deadline(
                        follow(robot.follower(), path),
                        robot.shooter.autoAim()))
                // endPose, not hold(follower): that one grab start pose.
                .then(hold(robot.follower(), path.endPose()));
    }

    /** Drive + intake + aim. Stop rollers, hold end pose. */
    public static Command driveIntaking(Robot robot, Path path) {
        return deadline(
                follow(robot.follower(), path),
                robot.intake.intake(),
                robot.shooter.autoAim())
                .then(robot.intake.stopIntake())
                // endPose, not hold(follower): that one grab start pose.
                .then(hold(robot.follower(), path.endPose()));
    }
}
