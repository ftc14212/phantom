package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Metrobotics pos tracking with odometry.
 * Track the robot's position using telemetry for ease of points.
 * @author David Grieas - 14212 MetroBotics
 * @version 1.3, 5/26/25
 **/

@Config
@Configurable
@Autonomous(name = "Position tracking odometry", group = "tools_ftc14212")
public class posTracking extends OpMode {
    public static double startPosX = 18;
    public static double startPosY = 119;
    public static double startPosRotation = 144;
    private final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos
    private Follower follower;
    static TelemetryManager telemetryM;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPos);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        // telemetry for debugging
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.addData("total heading", Math.toDegrees(follower.getTotalHeading()));
        // telemetry again
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading()));
        telemetry.update();
        follower.update();
    }
}