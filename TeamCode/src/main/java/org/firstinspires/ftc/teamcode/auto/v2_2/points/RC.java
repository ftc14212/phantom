package org.firstinspires.ftc.teamcode.auto.v2_2.points;

import com.pedropathing.geometry.Pose;

public class RC {
    public static final Pose startPose = new Pose(126, 119,Math.PI - Math.toRadians(144));
    public static final Pose shootClosePose = new Pose(90, 85, Math.toRadians(360));
    public static final Pose intakeClosePose = new Pose(116, 85, Math.toRadians(360));
    public static final Pose intakeMidPose = new Pose(116.5, 59, Math.toRadians(360));
    public static final Pose intakeMidControlPose = new Pose(69, 56, Math.toRadians(360));
    public static final Pose intakeFarPose = new Pose(118, 35, Math.toRadians(360));
    public static final Pose intakeFarControlPose = new Pose(63.5, 26, Math.toRadians(360));
    public static final Pose intakeGatePose = BC.intakeGatePose.mirror();
    // public static final Pose intakeGateControlPose = BC.intakeGateControlPose.mirror();
    public static final Pose parkPose = BC.parkPose.mirror();
}
