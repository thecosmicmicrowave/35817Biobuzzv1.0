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
 * Example auto: drive to shoot spot aiming, shoot all.
 *
 * <p>Pattern for every auto. Routine write once here, both alliances. Blue/Red subclass pick
 * alliance + paths. Movement from visualizer export. Actions in routine().
 */
public abstract class ExampleAuto extends NextOpMode {

    private final Robot robot;
    private final Alliance alliance;
    private final ExampleRoute route;

    /** False = no follower. Nothing scheduled. */
    private final boolean ready;

    protected ExampleAuto(Robot robot, Alliance alliance, ExampleRoute route) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        this.route = route;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.bindHardware();
        // Start pose = first path start, heading too.
        Pose startPose = route.driveToShoot().get(0);
        ready = robot.beginAuto(hardwareMap, alliance, startPose);
        robot.reset();
    }

    /** Routine. Say every mechanism action, defaults no run. */
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
        // Cut flywheel direct. Scheduler shutting down.
        robot.shooter.coastFlywheel();
    }
}
