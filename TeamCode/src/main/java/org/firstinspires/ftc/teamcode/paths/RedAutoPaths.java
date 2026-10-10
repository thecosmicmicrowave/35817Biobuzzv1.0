package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

// Visualizer export. Never hand-edit, re-export.
public class RedAutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(19.2305, 6.8101, 180);
    private final Pose path1Start = poseFactory.of(19.2305, 6.8101, 180);
    private final Pose path1 = poseFactory.of(7.5, 7, 180);
    private final Pose point2 = poseFactory.of(47.6751, 117.7702, 70.0649);
    private final Pose point2Segment1Heading = poseFactory.of(47.6751, 117.7702, 180);
    private final Pose point3 = poseFactory.of(47.9705, 127.7551, 88.3051);
    private final Pose point4 = poseFactory.of(14.431, 46.8114, -180);
    private final Pose point4Control1 = poseFactory.of(38.5707, 51.1633, 0);
    private final Pose point4Segment1Start = poseFactory.of(14.431, 46.8114, 90);
    private final Pose point4Segment1End = poseFactory.of(14.431, 46.8114, 265);
    private final Pose point4Segment3Start = poseFactory.of(14.431, 46.8114, -122.0153);
    private final Pose point4Segment3End = poseFactory.of(14.431, 46.8114, 180);
    private final Pose point5 = poseFactory.of(7.868, 102.9522, 90.6193);
    private final Pose point5Control1 = poseFactory.of(8.2374, 65.9972, 0);

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).heading(Interpolator.piecewise().until(0.0262, Interpolator.constant(point2Segment1Heading)).until(1, Interpolator.tangent));
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).heading(Interpolator.piecewise().until(0.1559, Interpolator.linear(point4Segment1Start, point4Segment1End)).until(0.7747, Interpolator.tangent).until(1, Interpolator.linear(point4Segment3Start, point4Segment3End)));
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).tangent();
    }
}
