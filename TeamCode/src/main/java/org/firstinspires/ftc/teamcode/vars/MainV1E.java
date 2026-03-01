package org.firstinspires.ftc.teamcode.vars;

import com.pedropathing.geometry.Pose;

public class MainV1E {
    // none
    public enum Alliance {
        RED,
        BLUE
    }
    public enum StartPos {
        CLOSE,
        FAR
    }
    public static Pose lastAutoPos = null;
    public static double lastTurretPos = -999;
    public static boolean redSideS = false;
}
