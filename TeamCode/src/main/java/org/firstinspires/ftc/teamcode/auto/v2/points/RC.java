package org.firstinspires.ftc.teamcode.auto.v2.points;

import com.pedropathing.geometry.Pose;

public class RC {
    public static final Pose startPose = BC.startPose.mirror();
    public static final Pose shootClosePose = BC.shootClosePose.mirror();
    public static final Pose intakeClosePose = new Pose(124, 83.4, Math.toRadians(0));
    public static final Pose intakeMidPose = new Pose(125.5, 59, Math.toRadians(0));
    public static final Pose intakeMidControlPose = new Pose(79, 56, Math.toRadians(0));
    public static final Pose intakeFarPose = new Pose(124, 35, Math.toRadians(0));
    public static final Pose intakeFarControlPose = new Pose(69.5, 26, Math.toRadians(0));
    public static final Pose intakeGatePose = BC.intakeGatePose.mirror();
    // public static final Pose intakeGateControlPose = BC.intakeGateControlPose.mirror();
    public static final Pose parkPose = BC.parkPose.mirror();
}
