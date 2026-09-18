package org.firstinspires.ftc.teamcode.auto.offseason.points;

import com.pedropathing.geometry.Pose;

public class Points {
    public static class BC {
        public static final Pose start = new Pose(18, 119, Math.toRadians(144));
        public static final Pose shootPre = new Pose(51, 91.7, Math.toRadians(140));
        public static final Pose intakeClose = new Pose(17.7, 83.2, Math.toRadians(180));
        public static final Pose shootClose = new Pose(67, 75, Math.toRadians(140));
        public static final Pose intakeMid = new Pose(10, 60, Math.toRadians(180));
        public static final Pose intakeMidControl = new Pose(55.6, 56.8);
        public static final Pose gate = new Pose(20, 69.8, Math.toRadians(180));
        public static final Pose gateControl = new Pose(29, 57.5);
        public static final Pose shootMid = shootClose;
        public static final Pose intakeFar = new Pose(14, 36, Math.toRadians(180));
        public static final Pose intakeFarControl = new Pose(73.6, 29.2);
        // public static final Pose shootFar = new Pose(67, 75, Math.toRadians(140));
        public static final Pose shootFar = shootClose;
        public static final Pose park = new Pose(22, 69.6, Math.toRadians(-90));
    }
    public static class RC {
        public static final Pose start = new Pose(126, 119,Math.PI - Math.toRadians(144));
        public static final Pose shootPre = new Pose(00, 00, Math.toRadians(144));
        public static final Pose intakeClose = new Pose(00, 00, Math.toRadians(180));
        public static final Pose shootClose = new Pose(00, 00, Math.toRadians(160));
        public static final Pose intakeMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeMidControl = new Pose(00, 00);
        public static final Pose gate = new Pose(00, 00, Math.toRadians(180));
        public static final Pose gateControl = new Pose(00, 00);
        public static final Pose shootMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeFar = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeFarControl = new Pose(00, 00);
        public static final Pose shootFar = shootMid;
        public static final Pose park = new Pose(00, 00, Math.toRadians(-90));
    }
    public static class BF {
        public static final Pose start = new Pose(54.6, 7.8, Math.toRadians(180));
        public static final Pose shootPre = new Pose(00, 00, Math.toRadians(144));
        public static final Pose intakeFar = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeFarControl = new Pose(00, 00);
        public static final Pose shootFar = new Pose(00, 00, Math.toRadians(160));
        public static final Pose intakeMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeMidControl = new Pose(00, 00);
        public static final Pose gate = new Pose(00, 00, Math.toRadians(180));
        public static final Pose gateControl = new Pose(00, 00);
        public static final Pose shootMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeClose = new Pose(00, 00, Math.toRadians(180));
        public static final Pose shootClose = new Pose(00, 00, Math.toRadians(180));
        public static final Pose park = new Pose(00, 00, Math.toRadians(-90));
        public static final Pose leave = new Pose(00, 00, Math.toRadians(-90));
    }
    public static class RF {
        public static final Pose start = BF.start.mirror();
        public static final Pose shootPre = new Pose(00, 00, Math.toRadians(144));
        public static final Pose intakeFar = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeFarControl = new Pose(00, 00);
        public static final Pose shootFar = new Pose(00, 00, Math.toRadians(160));
        public static final Pose intakeMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeMidControl = new Pose(00, 00);
        public static final Pose gate = new Pose(00, 00, Math.toRadians(180));
        public static final Pose gateControl = new Pose(00, 00);
        public static final Pose shootMid = new Pose(00, 00, Math.toRadians(180));
        public static final Pose intakeClose = new Pose(00, 00, Math.toRadians(180));
        public static final Pose shootClose = new Pose(00, 00, Math.toRadians(180));
        public static final Pose park = new Pose(00, 00, Math.toRadians(-90));
        public static final Pose leave = BF.leave.mirror();
    }
}
