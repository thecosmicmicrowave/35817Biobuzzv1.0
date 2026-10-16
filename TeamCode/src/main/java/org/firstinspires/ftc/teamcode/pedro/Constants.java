package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-3.14);
        c.yPodOffset.set(-3.72);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.021190333863466);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("rf");
        c.frontRightName.set("lf");
        c.backLeftName.set("rr");
        c.backRightName.set("lr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21751391044246157);
                Controller secondaryTranslationalForward = Controller.proportional(0.08036557782975995);
                Controller primaryTranslationalLateral = Controller.proportional(0.32758953546674363);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12103558000136128);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010901239276553799));
                c.brake.set(Controller.proportionalFeedforward(0.009266053385070729));

                c.headingFeedback.set(Controller.proportional(4.2853946303712025));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04367501236886864, 0.007069672950851767));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07909611684406867, 0.07208622289265268));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011005022330710928, 0.0011390208626559986));

                c.maxAchievableForwardVelocity.set(92.24415105760804);
                c.maxAchievableStrafeVelocity.set(77.97961205000547);
                c.naturalForwardDeceleration.set(33.779461623670954);
                c.naturalStrafeDeceleration.set(68.09493086080367);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}