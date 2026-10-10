package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Auto", preselectTeleop = "Main TeleOp")
public class BlueAuto extends MainAuto {
    public BlueAuto(Robot robot) {
        super(robot, Alliance.BLUE, new AutoRoute.Blue());
    }
}
