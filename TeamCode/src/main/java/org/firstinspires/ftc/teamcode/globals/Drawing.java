package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

/** Draws robot on Panels field. */
public final class Drawing {

    /** Robot circle radius, inches. */
    public static final double ROBOT_RADIUS = 9;

    private static final FieldManager FIELD = PanelsField.INSTANCE.getField();
    private static final Style ROBOT_STYLE = new Style("", "#3F51B5", 0.75);

    private Drawing() {
    }

    /** Draw robot: circle + line to front. Send to Panels. */
    public static void drawRobot(Pose pose) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }

        // Pedro coordinates. Set every frame, update() reset it.
        FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        FIELD.setStyle(ROBOT_STYLE);
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.circle(ROBOT_RADIUS);

        double frontX = pose.x() + ROBOT_RADIUS * Math.cos(pose.heading());
        double frontY = pose.y() + ROBOT_RADIUS * Math.sin(pose.heading());
        FIELD.moveCursor(pose.x(), pose.y());
        FIELD.line(frontX, frontY);

        FIELD.update();
    }
}
