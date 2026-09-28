package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Fake export. Replace with real one.
public class BlueExamplePaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose shootPose = poseFactory.of(56, 36, 90);

    public Path driveToShoot() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }
}
