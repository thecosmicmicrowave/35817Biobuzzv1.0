package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;

/**
 * Example autonomous: drive to a shooting spot while aiming, then empty the robot into the goal.
 *
 * <p>The pattern every auto follows. The routine is written once here, independent of alliance;
 * {@link BlueExampleAuto} and {@link RedExampleAuto} are one-line entries that pick the alliance
 * and the matching exported paths. Movement comes entirely from the visualizer export, and every
 * robot action is spelled out in {@link #routine()}.
 */
public abstract class ExampleAuto extends NextOpMode {

    private final Robot robot;
    private final Alliance alliance;
    private final ExampleRoute route;

    /** False when the follower could not be built, in which case nothing is scheduled. */
    private final boolean ready;

    protected ExampleAuto(Robot robot, Alliance alliance, ExampleRoute route) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        this.route = route;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.bindHardware();
        // The start pose is where the first path begins, heading included, so no coordinate is
        // duplicated outside the export.
        Pose startPose = route.driveToShoot().get(0);
        ready = robot.beginAuto(hardwareMap, alliance, startPose);
        robot.reset();
    }

    /**
     * The routine. Mechanism default commands do not run inside a sequential group, so every
     * step says what each mechanism does — see {@link AutoCommands}.
     */
    private Command routine() {
        return sequential(
                AutoCommands.driveAiming(robot, route.driveToShoot()),
                ShootCommands.shootAll(robot));
    }

    @Override
    public void start() {
        if (ready) {
            routine().schedule();
        }
    }

    @Override
    public void disabledPeriodic() {
        pushTelemetry();
    }

    @Override
    public void periodic() {
        pushTelemetry();
    }

    private void pushTelemetry() {
        Telemetry.log("alliance", alliance);
        if (!ready) {
            Telemetry.log("NOT READY", "follower unavailable — implement pedro/Constants.create()");
            return;
        }
        Pose pose = robot.pose();
        Telemetry.log("x", String.format("%.1f", pose.x()));
        Telemetry.log("y", String.format("%.1f", pose.y()));
        Telemetry.log("heading (deg)", String.format("%.1f", Math.toDegrees(pose.heading())));
    }

    @Override
    public void end() {
        // Cut the flywheel directly; the scheduler is being torn down, so a command would not run.
        robot.shooter.coastFlywheel();
    }
}
