package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueExamplePaths;
import org.firstinspires.ftc.teamcode.paths.RedExamplePaths;

/** Paths ExampleAuto use. */
public interface ExampleRoute {

    // One method per path, same name as visualizer.
    Path driveToShoot();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Blue extends BlueExamplePaths implements ExampleRoute {
    }

    final class Red extends RedExamplePaths implements ExampleRoute {
    }
}
