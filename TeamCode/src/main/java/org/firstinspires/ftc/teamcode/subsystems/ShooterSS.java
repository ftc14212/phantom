/***
 * TurretSS
 * @author David Grieas - 14212 MetroBotics
 * Turret susbsystem
 * started coding at 1/23/25  @  10:13 am
 * finished coding at 1/26/25  @  11:51 am
***/
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class ShooterSS extends SubsystemBase {
    // -------------------------
    // VALUES STUFF
    // -------------------------
    double shooterVelo = 0;
    double hoodCpos = 0;
    double shooterCpos = 0;
    double distShooter = 0;
    double offset = 0;

    // POSES
    Pose bluePos = new Pose(11, 137, 0);
    Pose redPos = new Pose(133, 137, 0);
    Pose target = bluePos;

    // INTERPLUTS
    InterpLUT shooterVeloLut = new InterpLUT();
    InterpLUT hoodLut = new InterpLUT();

    // -------------------------
    // FLIPPIN HARDWARE STUFF
    // -------------------------
    PIDFCoefficients pid;
    CombinedDcMotorEx shooter;
    CombinedServo hood;
    Follower follower = null;

    // --------------------------------
    // random okay leave me alone :C
    // --------------------------------
    boolean redSide = false;
    boolean shooterOn = false;
    double sLutMin = 0;
    double sLutMax = 0;
    double hLutMin = 0;
    double hLutMax = 0;

    // -------------------------
    // INIT THINGY
    // -------------------------
    public ShooterSS(PIDFCoefficients pid, CachingDcMotorEx shooter, CachingServo hood) {
        this.pid = pid;
        this.shooter = new CombinedDcMotorEx(shooter);
        this.hood = new CombinedServo(hood);
    }
    public ShooterSS(PIDFCoefficients pid, CachingDcMotorEx shooter, CachingServo hoodL, CachingServo hoodR) {
        this.pid = pid;
        this.shooter = new CombinedDcMotorEx(shooter);
        this.hood = new CombinedServo(hoodL, hoodR);
    }
    public ShooterSS(PIDFCoefficients pid, CachingDcMotorEx shooterL, CachingDcMotorEx shooterR, CachingServo hood) {
        this.pid = pid;
        this.shooter = new CombinedDcMotorEx(shooterL, shooterR);
        this.hood = new CombinedServo(hood);
    }
    public ShooterSS(PIDFCoefficients pid, CachingDcMotorEx shooterL, CachingDcMotorEx shooterR, CachingServo hoodL, CachingServo hoodR) {
        this.pid = pid;
        this.shooter = new CombinedDcMotorEx(shooterL, shooterR);
        this.hood = new CombinedServo(hoodL, hoodR);
    }

    // getters
    public double getCurrentVelocity() {
        return shooterCpos;
    }
    public double getTargetVelocity() {
        return shooterVelo;
    }
    public double getHoodPosition() {
        return hoodCpos;
    }
    public PIDFCoefficients getPid() {
        return pid;
    }
    public Pose getTargetPose() {
        return target;
    }
    public double getShooterVeloLut(double distShooter) {
        return shooterVeloLut.get(Math.max(sLutMin + 0.1, Math.min(sLutMax - 0.1, distShooter)));
    }
    public double getHoodLut(double distShooter) {
        return hoodLut.get(Math.max(hLutMin + 0.1, Math.min(hLutMax - 0.1, distShooter)));
    }//update the axon no no worky
    // -------------------------------------------------------------
    // METHODSSSS
    // -------------------------------------------------------------
    public void update(Follower follower) {
        // pose
        target = redSide ? redPos : bluePos;
        // vars
        this.follower = follower;
        distShooter = redSide ? Math.sqrt(Math.pow((redPos.getX() - follower.getPose().getX()), 2) + Math.pow((redPos.getY() - follower.getPose().getY()), 2)) : Math.sqrt(Math.pow((bluePos.getX() - follower.getPose().getX()), 2) + Math.pow((bluePos.getY() - follower.getPose().getY()), 2));
        distShooter += offset;
        shooterCpos = shooter.getVelocity();
        // update pid
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        // shooter code
        hood.setPosition(hoodCpos);
        shooter.setVelocity(shooterVelo);
    }
    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }
    public void setShooterOffset(double offset) {
        this.offset = offset;
    }
    public void setPoses(Pose bluePos, Pose redPos) {
        this.bluePos = bluePos;
        this.redPos = redPos;
    }
    public void setPoses(InterpLUT shooterVeloLut, double sLutMin, double sLutMax, InterpLUT hoodLut, double hLutMin, double hLutMax) {
        this.shooterVeloLut = shooterVeloLut;
        this.hoodLut = hoodLut;
        this.sLutMin = sLutMin;
        this.sLutMax = sLutMax;
        this.hLutMin = hLutMin;//ur not the meta david ur a loswrt
        this.hLutMax = hLutMax;
    }
    public void shooterOn(boolean shooterOn) {
        this.shooterOn = shooterOn;
    }
    public void alignShooter() {
        shooterVelo = shooterOn ? getShooterVeloLut(distShooter) : 0;
        hoodCpos = shooterOn ? getHoodLut(distShooter) : 0;
    }
    public void updatePID(PIDFCoefficients pid) {
        this.pid = pid;
    }
    // -------------------------
    // TELEMETRY FOR DEBUGGING MY AHH
    // -------------------------
    public String telemetry() {
        return("===== Shooter Telemetry =====\n" +
                "-- Positions --\n" +
                "Shooter current pos: " + shooterCpos + "\n" +
                "Hood current pos: " + hoodCpos + "\n" +
                "Shooter target pos: " + shooterVelo + "\n" +
                "-- PID Values --\n" +
                "P: " + pid.p + "\n" +
                "I: " + pid.i + "\n" +
                "D: " + pid.d + "\n" +
                "F: " + pid.f + "\n" +
                "-- Values --\n" +
                "Shooter redSide: " + redSide + "\n" +
                "Align hood: " + getHoodLut(distShooter) + "\n" +
                "Align shooter: " + getShooterVeloLut(distShooter) + "\n" +
                "Offset: " + offset + "\n" +
                "Shooter On: " + shooterOn + "\n" +
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
}
