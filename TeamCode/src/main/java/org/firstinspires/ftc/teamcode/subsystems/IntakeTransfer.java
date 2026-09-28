package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.globals.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/** Intake. Two stages, gate, flower arm. */
public class IntakeTransfer implements Mechanism {

    private final NextMotor stage1 = new NextMotor("stage1");
    private final NextMotor stage2 = new NextMotor("stage2");
    // Not final. bindServos() replace each OpMode.
    private NextServo gate;
    private NextServo flowerLeft;
    private NextServo flowerRight;

    private boolean gateOpen = false;
    private boolean flowerDeployed = false;

    public IntakeTransfer() {
        bindServos();
    }

    /** Make fresh gate + flower servos. Call every OpMode start. */
    public void bindServos() {
        gate = new NextServo("gate");
        flowerLeft = new NextServo("flowerLeft");
        flowerRight = new NextServo("flowerRight");
        flowerRight.setDirection(NextMotor.Direction.REVERSE);
    }

    // ---------------------------------------------------------------- lifecycle

    /** Known state. Motors off, gate closed, arm stowed. Flags match hardware. */
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

    /** Both stages in, gate closed. Artifacts stack on gate. */
    public Command intake() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(Constants.STAGE2_INTAKE_POWER);
        });
    }

    /** Stage 1 only. Stage 2 full. */
    public Command intakeStage1Only() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(0);
        });
    }

    /** Both stages reverse. Clear jam. */
    public Command eject() {
        return infinite(() -> {
            stage1.setThrottle(Constants.STAGE1_EJECT_POWER);
            stage2.setThrottle(Constants.STAGE2_EJECT_POWER);
        });
    }

    /** Default command. Motors off only, never touch servos. */
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

    /** Gate open, stage 2 push artifacts into flywheel. */
    public Command feed() {
        return infinite(() -> {
            applyGate(true);
            stage2.setThrottle(Constants.STAGE2_FEED_POWER);
        });
    }

    /** Gate open, both stages run. Empty robot. */
    public Command volley() {
        return infinite(() -> {
            applyGate(true);
            stage1.setThrottle(Constants.STAGE1_INTAKE_POWER);
            stage2.setThrottle(Constants.STAGE2_FEED_POWER);
        });
    }

    /** Stop both stages, gate untouched. */
    public Command stopIntake() {
        return instant(() -> {
            stage1.setThrottle(0);
            stage2.setThrottle(0);
        });
    }

    /** Close gate, stop stages. End of shot. */
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

        // flowerRight reversed in bindServos(). Both get same position.
        flowerLeft.setPosition(position);
        flowerRight.setPosition(position);
    }
}
