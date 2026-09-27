package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Stand-in for a Pedro visualizer export, handwritten in the exact format the visualizer's
// "paths only" export produces. Replace it with a real export; do not hand-edit real exports.
public class BlueExamplePaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose shootPose = poseFactory.of(56, 36, 90);

    public Path driveToShoot() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }
}
