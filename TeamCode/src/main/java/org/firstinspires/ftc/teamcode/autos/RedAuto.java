package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Auto", preselectTeleop = "Main TeleOp")
public class RedAuto extends MainAuto {
    public RedAuto(Robot robot) {
        super(robot, Alliance.RED, new AutoRoute.Red());
    }
}
