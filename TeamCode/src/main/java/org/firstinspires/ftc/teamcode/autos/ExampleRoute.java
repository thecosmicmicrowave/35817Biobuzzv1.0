package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueExamplePaths;
import org.firstinspires.ftc.teamcode.paths.RedExamplePaths;

/**
 * Paths ExampleAuto use. One type for both alliance exports.
 *
 * <p>Adapters inherit export methods. Path renamed in visualizer = compile error here.
 *
 * <p>Add path: draw + name in visualizer, re-export both, add method here same name.
 */
public interface ExampleRoute {

    Path driveToShoot();

    final class Blue extends BlueExamplePaths implements ExampleRoute {
    }

    final class Red extends RedExamplePaths implements ExampleRoute {
    }
}
