package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/**
 * Two-stage intake and transfer, the gate that holds artifacts back from the flywheel, and the
 * flower arm.
 *
 * <p>The default command zeroes the motors and touches nothing else. This is deliberate: an
 * instant command completes and releases the mechanism on the frame it runs, so the default
 * resumes immediately afterwards. A default that also asserted "gate closed, arm stowed" would
 * undo {@link #openGate()} and {@link #deployFlower()} one frame after they were issued. Servo
 * state is set once and left alone.
 */
public class IntakeTransfer implements Mechanism {

    private final NextMotor stage1 = new NextMotor("stage1");
    private final NextMotor stage2 = new NextMotor("stage2");
    // Not final: replaced at the start of every OpMode by bindServos().
    private NextServo gate;
    private NextServo flowerLeft;
    private NextServo flowerRight;

    private boolean gateOpen = false;
    private boolean flowerDeployed = false;

    public IntakeTransfer() {
        bindServos();
    }

    /**
     * Builds fresh servo objects for the gate and flower arm.
     *
     * <p>Must run at the start of every OpMode. At each OpMode start the SDK resets every servo:
     * {@code LynxController.resetDeviceConfigurationForOpMode()} calls {@code initializeHardware()},
     * which puts the pulse range back to the 600-2400 us default and disables PWM, and
     * {@code ServoImpl.resetDeviceConfigurationForOpMode()} puts the direction back to forward.
     * This mechanism outlives OpModes, so servo objects built once would configure the hardware
     * only in the first OpMode after the app starts. Fresh {@code NextServo}s re-apply the range
     * and direction on first use, and have an empty position cache, so their first position write
     * always reaches the hardware instead of being skipped as unchanged while the servo sits
     * disabled. Servos do not register in any shared loop, so replacing them leaks nothing.
     *
     * <p>Without this, the reversal of flowerRight would be lost after the first OpMode and the
     * two flower servos would drive the arm in opposite directions.
     */
    public void bindServos() {
        gate = new NextServo("gate");
        flowerLeft = new NextServo("flowerLeft");
        flowerRight = new NextServo("flowerRight");
        flowerRight.setDirection(NextMotor.Direction.REVERSE);
    }

    // ---------------------------------------------------------------- lifecycle

    /**
     * Returns the intake to a known state and resyncs the cached servo flags with the hardware.
     * Without this, auto ending with the flower arm deployed would leave teleop believing the
     * arm is deployed while it sits wherever auto left it, sending the first toggle the wrong way.
     */
    public void reset() {
        stage1.setThrottle(0);
        stage2.setThrottle(0);
        applyGate(false);
        applyFlower(false);
    }

    @Override
    public Command getDefaultCommand() {
        return stopRollers();
    }

    // ---------------------------------------------------------------- state

    public boolean isGateOpen() {
        return gateOpen;
    }

    public boolean isFlowerDeployed() {
        return flowerDeployed;
    }

    // ---------------------------------------------------------------- commands

    /** Both stages forward with the gate closed, so artifacts stage against the gate. */
    public Command intake() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(Constants.STAGE2_INTAKE_POWER);
        });
    }

    /** Stage 1 only, for when stage 2 is already loaded. */
    public Command intakeStage1Only() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(0);
        });
    }

    /** Both stages reversed, for clearing a jam. */
    public Command eject() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_EJECT_POWER);
            stage2.setThrottle(Constants.STAGE2_EJECT_POWER);
        });
    }

    /** Default command. Motors only — see the class comment. */
    public Command stopRollers() {
        return infinite(() -> {
            stage1.setThrottle(0);
            stage2.setThrottle(0);
        });
    }

    public Command openGate() {
        return instant(() -> applyGate(true));
    }

    public Command closeGate() {
        return instant(() -> applyGate(false));
    }

    /** Gate open and stage 2 running, pushing artifacts into the flywheel. */
    public Command feed() {
        return infinite(() -> {
            applyGate(true);
            stage2.setThrottle(Constants.STAGE2_FEED_POWER);
        });
    }

    /**
     * Gate open with both stages running, emptying the robot into the goal.
     *
     * <p>A single command rather than {@code feed().with(intake())}: those both require this
     * mechanism, and composing two commands that require the same mechanism into one parallel
     * group makes them conflict with each other.
     */
    public Command volley() {
        return infinite(() -> {
            applyGate(true);
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(Constants.STAGE2_FEED_POWER);
        });
    }

    /**
     * Stops both stages and leaves the gate alone. Needed in auto, where this mechanism's default
     * command never runs, so nothing else would stop the rollers after an intake step ends.
     */
    public Command stopIntake() {
        return instant(() -> {
            stage1.setThrottle(0);
            stage2.setThrottle(0);
        });
    }

    /** Closes the gate and stops the feed. Used to end a shot sequence. */
    public Command stopFeed() {
        return instant(() -> {
            applyGate(false);
            stage1.setThrottle(0);
            stage2.setThrottle(0);
        });
    }

    public Command deployFlower() {
        return instant(() -> applyFlower(true));
    }

    public Command stowFlower() {
        return instant(() -> applyFlower(false));
    }

    public Command toggleFlower() {
        return instant(() -> applyFlower(!flowerDeployed));
    }

    // ---------------------------------------------------------------- hardware

    private void applyGate(boolean open) {
        gateOpen = open;
        gate.setPosition(open ? Constants.GATE_OPEN_POSITION : Constants.GATE_CLOSED_POSITION);
    }

    private void applyFlower(boolean deployed) {
        flowerDeployed = deployed;
        double position = deployed
                ? Constants.FLOWER_DEPLOYED_POSITION
                : Constants.FLOWER_STOWED_POSITION;

        // flowerRight is reversed in the constructor, so both take the same commanded position.
        flowerLeft.setPosition(position);
        flowerRight.setPosition(position);
    }
}
