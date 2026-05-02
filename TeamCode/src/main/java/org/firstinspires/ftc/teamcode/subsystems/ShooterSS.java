/***
 * ShooterSS
 * @author David Grieas - 14212 MetroBotics
 * Shooter susbsystem
***/
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.enums.ShooterS;
import org.firstinspires.ftc.teamcode.utils.CombinedDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;

public class ShooterSS extends SubsystemBase {
    // hardware
    private final CombinedDcMotorEx motors;
    private final CombinedServo hood;
    private final Servo leds;
    // data
    private ShooterS status;
    private InterpLUT shooterLUT;
    private InterpLUT hoodLUT;
    private double target = 0;
    private double hoodCpos = 0;
    private double distShooter = 0;
    private Follower follower = null;
    private Pose redPos = new Pose();
    private Pose bluePos = new Pose();
    private Pose targetPos = bluePos;
    private boolean redSide = false;
    // config
    private int tolerance = 100;
    final double LED_ERROR = 0.278;
    final double LED_TARGET = 1;
    final double LED_REV = 0.388;
    final double LED_REG = 0.611;
    private double sLutMin = 0;
    private double sLutMax = 0;
    private double hLutMin = 0;
    private double hLutMax = 0;
    private PIDFCoefficients pid;
    private double offset = 0;

    // init
    public ShooterSS(CombinedDcMotorEx motors, CombinedServo servos, Servo leds, PIDFCoefficients pid) {
        // variables
        this.motors = motors;
        this.hood = servos;
        this.leds = leds;
        this.status = ShooterS.STOPPED;
        this.pid = pid;
        // init
        motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // methods
    public void update(Follower follower) {
        targetPos = redSide ? redPos : bluePos;
        if (status != ShooterS.ERROR) {
            // emergency stop if something unplugged
            for(int i = 0; i < motors.getMotorsSize(); i++) if (status != ShooterS.STOPPED && motors.getVelocity(i) < 30 || motors.getCurrent(CurrentUnit.MILLIAMPS) == 0) status = ShooterS.ERROR;
            // status
            if (atTarget() && target != 0) status = ShooterS.AT_TARGET;
            if (target > motors.getVelocity() && !atTarget()) status = ShooterS.REVVING;
            if (motors.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT && target == 0) status = ShooterS.FLOATING;
            if (motors.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE && target == 0) status = ShooterS.BRAKING;
            if (motors.getVelocity() == 0) status = ShooterS.STOPPED;
            // led control
            if (status == ShooterS.AT_TARGET) leds.setPosition(LED_TARGET);
            if (status == ShooterS.REVVING) leds.setPosition(LED_REV);
            if (status == ShooterS.STOPPED) leds.setPosition(LED_REG);
            // update
            this.follower = follower;
            motors.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
            distShooter = Math.sqrt(Math.pow((targetPos.getX() - follower.getPose().getX()), 2) + Math.pow((targetPos.getY() - follower.getPose().getY()), 2));
            distShooter += offset;
            // shooter code
            hood.setPosition(hoodCpos);
            motors.setVelocity(target);
        } else {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setVelocity(0);
            leds.setPosition(LED_ERROR);
        }
    }

    public void align() {
        setTarget(getShooterVeloLut(distShooter));
        setHoodCpos(getHoodLut(distShooter));
    }

    // setters
    public void setTarget(double target) {
        this.target = target;
    }
    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }
    public void setHoodCpos(double hoodCpos) {
        this.hoodCpos = hoodCpos;
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void setPose(Pose redPos, Pose bluePos) {
        this.redPos = redPos;
        this.bluePos = bluePos;
    }
    public void setPoses(InterpLUT shooterLUT, double sLutMin, double sLutMax, InterpLUT hoodLUT, double hLutMin, double hLutMax) {
        this.shooterLUT = shooterLUT;
        this.hoodLUT = hoodLUT;
        this.sLutMin = sLutMin;
        this.sLutMax = sLutMax;
        this.hLutMin = hLutMin;
        this.hLutMax = hLutMax;
    }
    public void setZeroPower(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motors.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void updatePID(PIDFCoefficients pid) {
        this.pid = pid;
    }
    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }
    // getters
    public boolean atTarget() {
        return motors.getVelocity() >= target - tolerance && motors.getVelocity() <= target + tolerance;
    }
    public int getTolerance() {
        return tolerance;
    }
    public double getTarget() {
        return target;
    }
    public double getHoodCpos() {
        return hoodCpos;
    }
    public PIDFCoefficients getPid() {
        return pid;
    }
    public double getVelocity() {
        return motors.getVelocity();
    }
    public Pose getTargetPos() {
        return targetPos;
    }
    public double getShooterVeloLut(double distShooter) {
        return shooterLUT.get(Math.max(sLutMin + 0.1, Math.min(sLutMax - 0.1, distShooter)));
    }
    public double getHoodLut(double distShooter) {
        return hoodLUT.get(Math.max(hLutMin + 0.1, Math.min(hLutMax - 0.1, distShooter)));
    }
    public ShooterS getStatus() {
        return status;
    }
    // telemetry
    public String telemetry() {
        return("===== Shooter Telemetry =====\n" +
                "-- Positions --\n" +
                "Shooter velocity: " + getVelocity() + "\n" +
                "Hood current pos: " + getHoodCpos() + "\n" +
                "Shooter target velocity: " + getTarget() + "\n" +
                "Distance from target: " + distShooter + "\n" +
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
                "Tolerance: " + getTolerance() + "\n" +
                "Status: " + getStatus() + "\n" +
                "-- Motor Data --\n" +
                "Shooter 1 velocity: " + motors.getVelocity(0) + "\n" +
                "Shooter 1 current: " + motors.getCurrent(0, CurrentUnit.MILLIAMPS) + "\n" +
                "Shooter 2 velocity: " + motors.getVelocity(1) + "\n" +
                "Shooter 2 current: " + motors.getCurrent(0, CurrentUnit.MILLIAMPS) + "\n" +
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
