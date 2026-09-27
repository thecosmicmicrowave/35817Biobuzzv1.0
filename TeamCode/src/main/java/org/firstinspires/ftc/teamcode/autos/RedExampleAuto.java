package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Example", group = "Example", preselectTeleop = "Main TeleOp")
public class RedExampleAuto extends ExampleAuto {
    public RedExampleAuto(Robot robot) {
        super(robot, Alliance.RED, new ExampleRoute.Red());
    }
}
