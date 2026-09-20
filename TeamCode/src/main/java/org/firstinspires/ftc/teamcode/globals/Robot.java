package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;


import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/**
 * The robot. NextFTC discovers this class by classpath scan and injects it into every OpMode
 * constructor, so there must be exactly one implementation of {@link NextRobot} with a public
 * no-argument constructor.
 *
 * <p>Deliberately no static instance. The scanner already hands the robot to every OpMode, and
 * a static would be actively harmful: {@code RobotScanner} clears its own reference and re-runs
 * on a Sloth reload or a robot reconfiguration, but a static assigned in this constructor would
 * not be cleared, leaving a stale object with dead hardware handles behind. Commands that need
 * the robot take it as a parameter.
 *
 * <p>This object is built once at event-loop creation and outlives individual OpMode runs, so
 * every OpMode must call {@link #initFollower(HardwareMap, Pose)} and {@link #reset()} — see {@link #reset()}.
 */
public class Robot implements NextRobot {

    public final NextMotor frontLeft = new NextMotor("frontLeft");
    public final NextMotor frontRight = new NextMotor("frontRight");
    public final NextMotor backLeft = new NextMotor("backLeft");
    public final NextMotor backRight = new NextMotor("backRight");

    public final IntakeTransfer intake = new IntakeTransfer();
    public final Shooter shooter = new Shooter(this::pose);

    private Follower follower;

    public Robot() {
        // Safe in a field-initializer context: NextMotor defers the write until the lazy
        // hardware resolves, so this does not touch the hardware map during the classpath scan.
        frontLeft.setDirection(Constants.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(Constants.BACK_LEFT_DIRECTION);
        frontRight.setDirection(Constants.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(Constants.BACK_RIGHT_DIRECTION);
    }

    // ---------------------------------------------------------------- lifecycle

    /**
     * Builds the Pedro follower and sets its starting pose. Every OpMode calls this in its
     * constructor — the follower is not rebuilt automatically between runs, so without it the
     * robot carries pose from the previous OpMode.
     *
     * <p>Takes the OpMode's {@code hardwareMap} rather than reading NextFTC's static one:
     * {@code RobotController} implements a Sinister interface that NextFTC does not expose on
     * the compile classpath, so referencing it here would not compile.
     */
    public void initFollower(HardwareMap hardwareMap, Pose startPose) {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
        }
    }

    /**
     * Returns both mechanisms to a known state. Called by every OpMode alongside
     * {@link #initFollower(HardwareMap, Pose)}, because mechanisms are constructed once at event-loop
     * creation and their cached state otherwise carries between runs.
     */
    public void reset() {
        shooter.reset();
        intake.reset();
    }

    @Override
    public void periodic() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return new HashSet<Mechanism>(Arrays.asList(shooter, intake));
    }

    // ---------------------------------------------------------------- accessors

    public Follower follower() {
        return follower;
    }

    /**
     * The robot's current field pose, or the origin before the follower exists. The shooter
     * reads this every loop, so it must not throw during init.
     */
    public Pose pose() {
        return follower == null ? Constants.DEFAULT_START_POSE : follower.pose();
    }
}
