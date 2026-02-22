/***
 * TurretSS
 * @author David Grieas - 14212 MetroBotics
 * Turret susbsystem
 * started coding at 1/23/25  @  4:13 pm
 * finished coding at 1/23/25  @  6:20 pm
***/
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;

public class TurretSS extends SubsystemBase {
    // -------------------------
    // VALUES STUFF
    // -------------------------
    int TPR;
    double ratio;
    double offset;
    public double turretTpos = 0;
    double turretCpos = 0;
    boolean turretOn = false;
    double lastTurretPos;

    Pose bluePos = new Pose(11, 137, 0);
    Pose redPos = new Pose(133, 137, 0);
    Pose target = bluePos;

    // -------------------------
    // FLIPPIN HARDWARE STUFF
    // -------------------------
    PIDController pid;
    CRServo turret;
    CombinedCRServo turretC;
    DcMotorEx encoder;
    Follower follower = null;

    // --------------------------------
    // random okay leave me alone :C
    // --------------------------------
    boolean redSide = false;
    double F;
    int minWrap = -210;
    int maxWrap = 190;
    boolean combined;

    // -------------------------
    // INIT THINGY
    // -------------------------
    public TurretSS(PIDController pid, double F, CRServo turret, DcMotorEx encoder, int TPR, double ratio) {
        this(pid, F, turret, encoder, TPR, ratio, 0);
    }
    public TurretSS(PIDController pid, double F, CRServo turret, DcMotorEx encoder, int TPR, double ratio, double offset) {
        this(pid, F, turret, encoder, TPR, ratio, offset, -999);
    }
    public TurretSS(PIDController pid, double F, CRServo turret, DcMotorEx encoder, int TPR, double ratio, double offset, double lastTurretPos) {
        // vars
        this.pid = pid;
        this.turret = turret;
        this.encoder = encoder;
        this.TPR = TPR;
        this.ratio = ratio;
        this.offset = offset;
        this.lastTurretPos = lastTurretPos;
        this.F = F;
        // init stuff
        if (lastTurretPos != -999) turretCpos = lastTurretPos;
        else encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        combined = false;
    }

    public TurretSS(PIDController pid, double F, CombinedCRServo turret, DcMotorEx encoder, int TPR, double ratio) {
        this(pid, F, turret, encoder, TPR, ratio, 0);
    }
    public TurretSS(PIDController pid, double F, CombinedCRServo turret, DcMotorEx encoder, int TPR, double ratio, double offset) {
        this(pid, F, turret, encoder, TPR, ratio, offset, -999);
    }
    public TurretSS(PIDController pid, double F, CombinedCRServo turret, DcMotorEx encoder, int TPR, double ratio, double offset, double lastTurretPos) {
        // vars
        this.pid = pid;
        this.turretC = turret;
        this.encoder = encoder;
        this.TPR = TPR;
        this.ratio = ratio;
        this.offset = offset;
        this.lastTurretPos = lastTurretPos;
        this.F = F;
        // init stuff
        if (lastTurretPos != -999) turretCpos = lastTurretPos;
        else encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        combined = true;
    }
    // getters
    public double getCurrentPos() {
        return turretCpos;
    }
    public double getTargetPos() {
        return turretTpos;
    }
    public boolean isTurretOn() {
        return turretOn;
    }
    public PIDController getPid() {
        return pid;
    }
    public Pose getTargetPose() {
        return target;
    }
    // -------------------------------------------------------------
    // METHODSSSS
    // -------------------------------------------------------------
    public void update(Follower follower) {
        this.follower = follower;
        // pose
        target = redSide ? redPos : bluePos;
        // grab current pos
        turretCpos = (-encoder.getCurrentPosition() / (TPR * ratio)) * 360;
        // turret code
        double error = wrap(turretTpos - turretCpos);
        double power = pid.calculate(error) + F;
        power = Math.max(-1, Math.min(1, power));
        if (combined) turretC.setPower(power);
        else turret.setPower(power);
    }
    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }
    public void setPoses(Pose bluePos, Pose redPos) {
        this.bluePos = bluePos;
        this.redPos = redPos;
    }
    public void turretOn(boolean turretOn) {
        this.turretOn = turretOn;
    }
    public double alignTurret() {
        return turretTpos = turretOn ? wrap(
                alignTurret(
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        Math.toDegrees(follower.getHeading()),
                        target
                )
        ) : 0;
    }
    public void updateTurretTpos(double turretTpos) {
        this.turretTpos = turretTpos;
    }
    public void setTurretOffset(double offset) {
        this.offset = offset;
    }
    public void setWrapAngles(int min, int max) {
        minWrap = min;
        maxWrap = max;
    }
    public void updatePID(PIDController pid, double F) {
        this.pid = pid;
        this.F = F;
    }
    // -------------------------
    // TELEMETRY FOR DEBUGGING MY AHH
    // -------------------------
    public String telemetry() {
        return("===== Turret Telemetry =====\n" +
                "-- Positions --\n" +
                "Turret current pos: " + turretCpos + "\n" +
                "Turret target pos: " + turretTpos + "\n" +
                "-- PID Values --\n" +
                "P: " + pid.getP() + "\n" +
                "I: " + pid.getI() + "\n" +
                "D: " + pid.getD() + "\n" +
                "F: " + F + "\n" +
                "-- Values --\n" +
                "Turret redSide: " + redSide + "\n" +
                "Align Turret output: " + wrap(alignTurret(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()), target)) + "\n" +
                "Offset: " + offset + "\n" +
                "Turret On: " + turretOn + "\n" +
                "Turret raw Power" + (turret == null ? turretC.getPower() : turret.getPower()) + "\n" +
                "-- Poses --\n" +
                "Follower:" + "\n" +
                "X: " + follower.getPose().getX() + "\n" +
                "Y: " + follower.getPose().getY() + "\n" +
                "heading: " + Math.toDegrees(follower.getHeading()) + "\n" +
                "\nRed pos:" + "\n" +
                "X: " + redPos.getPose().getX() + "\n" +
                "Y: " + redPos.getPose().getY() + "\n" +
                "heading: " + Math.toDegrees(redPos.getHeading()) + "\n" +
                "\nBlue pos:" + "\n" +
                "X: " + bluePos.getPose().getX() + "\n" +
                "Y: " + bluePos.getPose().getY() + "\n" +
                "heading: " + Math.toDegrees(bluePos.getHeading()) + "\n");
    }
    // -------------------------------------------------------------
    // HELPERSSS
    // -------------------------------------------------------------
    private double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return redSide ?  turretAngle + offset : turretAngle - offset;
    }
    // wrap code
    public double wrap(double angle) {
        if (angle > maxWrap) angle -= 360;
        if (angle < minWrap) angle += 360;
        return angle;
    }
}
