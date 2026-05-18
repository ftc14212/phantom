package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

/**
 * SOTM
 * turret + shooter prediction
 */
public class SOTM {
    // values
    public static double latency = 0.12;
    public static double alpha = 0.2;
    public static double t = 0.25;
    // filtered velocity
    private double vxFilt = 0;
    private double vyFilt = 0;
    // init
    public SOTM() {

    }

    /**
     * Update and filter robot velocity
     */
    public void updateVelocity(double vxRaw, double vyRaw) {
        vxFilt = alpha * vxRaw + (1 - alpha) * vxFilt;
        vyFilt = alpha * vyRaw + (1 - alpha) * vyFilt;
    }

    /**
     * Predict robot position after time t
     */
    public double[] predictPose(double robotX, double robotY, double t) {
        return new double[]{
                robotX + vxFilt * t,
                robotY + vyFilt * t
        };
    }

    /**
     * Compute turret angle (field locked)
     */
    public double computeTurretAngle(Pose robotPose, Pose targetPos) {
        double totalTime = t + latency;
        double[] pred = predictPose(robotPose.getX(), robotPose.getY(), totalTime);
        double dx = targetPos.getX() - pred[0];
        double dy = targetPos.getY() - pred[1];
        return Math.atan2(dy, dx);
    }

    /**
     * Predict robot field pose after time t using filtered velocity
     */
    public Pose predictRobotPose(Pose currentPose) {
        double x = currentPose.getX() + vxFilt * t;
        double y = currentPose.getY() + vyFilt * t;
        return new Pose(x, y, currentPose.getHeading());
    }

    /**
     * Compute shooter distance compensation
     */
    public double computeCompensatedDistance(Pose robotPose, Pose targetPos) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        double dx = targetPos.getX() - rx;
        double dy = targetPos.getY() - ry;
        double dist = Math.hypot(dx, dy);
        double shooterVel = ShooterSS.getShooterVeloLut(dist);
        double t = dist / shooterVel;
        t += latency;
        double[] pred = predictPose(rx, ry, t);
        double dx2 = targetPos.getX() - pred[0];
        double dy2 = targetPos.getY() - pred[1];
        return Math.hypot(dx2, dy2);
    }

    public double getVx() { return vxFilt; }
    public double getVy() { return vyFilt; }
}
