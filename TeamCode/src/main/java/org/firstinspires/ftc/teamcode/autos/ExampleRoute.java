package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueExamplePaths;
import org.firstinspires.ftc.teamcode.paths.RedExamplePaths;

/**
 * The paths {@link ExampleAuto} uses, as one type both alliances' exports can satisfy.
 *
 * <p>The visualizer's two exports share method names but no type, and one cannot be added
 * without editing generated code. The nested adapters inherit the generated methods and declare
 * this interface. If a path is renamed in the visualizer, the adapter stops compiling and the
 * error names the missing method — rather than an auto that silently breaks at a match.
 *
 * <p>To add a path: draw and name it in the visualizer, re-export both alliances, and add a
 * method here with the same name.
 */
public interface ExampleRoute {

    Path driveToShoot();

    final class Blue extends BlueExamplePaths implements ExampleRoute {
    }

    final class Red extends RedExamplePaths implements ExampleRoute {
    }
}
