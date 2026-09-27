package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Example", group = "Example", preselectTeleop = "Main TeleOp")
public class BlueExampleAuto extends ExampleAuto {
    public BlueExampleAuto(Robot robot) {
        super(robot, Alliance.BLUE, new ExampleRoute.Blue());
    }
}
