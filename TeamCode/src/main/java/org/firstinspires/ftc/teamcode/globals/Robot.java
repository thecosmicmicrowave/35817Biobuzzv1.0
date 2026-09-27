package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;


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
 * <p>The drivetrain is not represented here. Pedro's {@link Follower} owns those four motors
 * and teleop drives them through {@code follower.manual(...)}; adding NextFTC motors for the
 * same hardware would mean two writers fighting over it every loop.
 *
 * <p>This object is built once at event-loop creation and outlives individual OpMode runs, so
 * every OpMode must call {@link #initFollower(HardwareMap, Pose)} and {@link #reset()} — see {@link #reset()}.
 */
public class Robot implements NextRobot {

    public final IntakeTransfer intake = new IntakeTransfer();
    public final Shooter shooter = new Shooter(this::pose, this::alliance);

    private Follower follower;

    private Alliance alliance = Alliance.BLUE;

    /**
     * The follower's pose as of the latest loop, kept on the robot rather than read from the
     * follower so it survives the follower being rebuilt when the next OpMode starts.
     */
    private Pose lastPose = Constants.DEFAULT_START_POSE;

    /** Set when an auto starts, consumed by the next teleop. */
    private boolean autoHandoffPending = false;

    public Robot() {
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
     * Starts an autonomous run: sets the alliance, starts the follower at the auto's start pose,
     * and marks a handoff as pending so the next teleop picks up the alliance and wherever the
     * auto ends. Returns false if the follower is unavailable, which is the case until
     * {@code pedro/Constants.create()} is implemented.
     */
    public boolean beginAuto(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        this.alliance = alliance;
        initFollower(hardwareMap, startPose);
        lastPose = startPose;
        autoHandoffPending = follower != null;
        return follower != null;
    }

    /** Whether an auto has run since the app started and its handoff has not been used yet. */
    public boolean hasAutoHandoff() {
        return autoHandoffPending;
    }

    /**
     * The pose teleop should start from: where the last auto ended if its handoff is still
     * pending, otherwise the default start pose. Consumes the handoff, so a second teleop run
     * afterwards — a practice run, say — starts from the default instead of inheriting a stale
     * position.
     */
    public Pose takeHandoffPose() {
        Pose pose = autoHandoffPending ? lastPose : Constants.DEFAULT_START_POSE;
        autoHandoffPending = false;
        return pose;
    }

    /**
     * Rebuilds every servo object so its pulse range and direction are re-applied. Must be the
     * first thing every OpMode does, before anything writes a servo: the SDK resets servo range
     * and direction at the start of each OpMode, and the mechanisms outlive OpModes. See
     * {@code Shooter.bindServos()} for the details.
     */
    public void bindHardware() {
        shooter.bindServos();
        intake.bindServos();
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
            lastPose = follower.pose();
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

    public Alliance alliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * The robot's current field pose, or the origin before the follower exists. The shooter
     * reads this every loop, so it must not throw during init.
     */
    public Pose pose() {
        return follower == null ? Constants.DEFAULT_START_POSE : follower.pose();
    }
}
