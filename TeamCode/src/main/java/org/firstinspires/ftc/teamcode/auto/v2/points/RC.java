package org.firstinspires.ftc.teamcode.auto.v2.points;

import com.pedropathing.geometry.Pose;

public class RC {
    public static final Pose startPose = BC.startPose.mirror();
    public static final Pose shootClosePose = BC.shootClosePose.mirror();
    public static final Pose intakeClosePose = new Pose(124, 83.4, Math.toRadians(0));
    public static final Pose intakeMidPose = BC.intakeMidPose.mirror();
    public static final Pose intakeMidControlPose = BC.intakeMidControlPose.mirror();
    public static final Pose intakeFarPose = BC.intakeFarPose.mirror();
    public static final Pose intakeFarControlPose = BC.intakeFarControlPose.mirror();
    public static final Pose intakeGatePose = BC.intakeGatePose.mirror();
    // public static final Pose intakeGateControlPose = BC.intakeGateControlPose.mirror();
    public static final Pose parkPose = BC.parkPose.mirror();
}
