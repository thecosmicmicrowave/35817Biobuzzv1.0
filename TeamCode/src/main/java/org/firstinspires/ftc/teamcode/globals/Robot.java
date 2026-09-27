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
 * Robot. NextFTC find it, give to every OpMode. Only one, public no-arg constructor.
 *
 * <p>No static instance. Sloth reload make new robot, static keep old dead one. Pass robot
 * as parameter.
 *
 * <p>No drive motors here. Pedro Follower own them.
 *
 * <p>Built once, live across OpModes. Every OpMode call bindHardware(), initFollower(),
 * reset().
 */
public class Robot implements NextRobot {

    public final IntakeTransfer intake = new IntakeTransfer();
    public final Shooter shooter = new Shooter(this::pose, this::alliance);

    private Follower follower;

    private Alliance alliance = Alliance.BLUE;

    /** Latest pose. Kept here, survive follower rebuild next OpMode. */
    private Pose lastPose = Constants.DEFAULT_START_POSE;

    /** Auto set it, next teleop use it. */
    private boolean autoHandoffPending = false;

    public Robot() {
    }

    // ---------------------------------------------------------------- lifecycle

    /** Build Pedro follower, set start pose. Take OpMode hardwareMap. */
    public void initFollower(HardwareMap hardwareMap, Pose startPose) {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
        }
    }

    /**
     * Start auto: set alliance, start follower at start pose, mark handoff for teleop.
     * False = no follower (pedro/Constants.create() not done).
     */
    public boolean beginAuto(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        this.alliance = alliance;
        initFollower(hardwareMap, startPose);
        lastPose = startPose;
        autoHandoffPending = follower != null;
        return follower != null;
    }

    /** Auto ran, handoff not used yet. */
    public boolean hasAutoHandoff() {
        return autoHandoffPending;
    }

    /** Teleop start pose: where auto ended, else default. Use once, then gone. */
    public Pose takeHandoffPose() {
        Pose pose = autoHandoffPending ? lastPose : Constants.DEFAULT_START_POSE;
        autoHandoffPending = false;
        return pose;
    }

    /** Fresh servos, re-apply range + direction. First thing every OpMode. */
    public void bindHardware() {
        shooter.bindServos();
        intake.bindServos();
    }

    /** Both mechanisms to known state. */
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

    /** Robot pose now. Default before follower exist. Never throw. */
    public Pose pose() {
        return follower == null ? Constants.DEFAULT_START_POSE : follower.pose();
    }
}
