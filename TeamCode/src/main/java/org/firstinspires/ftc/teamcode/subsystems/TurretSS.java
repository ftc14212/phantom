/***
 * TurretSS
 * @author David Grieas - 14212 MetroBotics
 * Turret susbsystem
***/
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.enums.ShooterS;
import org.firstinspires.ftc.teamcode.subsystems.enums.TurretS;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;

public class TurretSS extends SubsystemBase {
    // hardware
    private final CombinedCRServo servos;
    private final DcMotorEx encoder;
    // data
    private double turretCpos = 0;
    private TurretS status;
    private double target = 0;
    private Follower follower = null;
    private Pose redPos = new Pose();
    private Pose bluePos = new Pose();
    private Pose targetPos = bluePos;
    private boolean redSide = false;
    // config
    private int tolerance = 10;
    private PIDController pid;
    private double offset = 0;
    int TPR = PIDTuneTurret.TPR;
    double ratio = PIDTuneTurret.ratio;
    int minWrap = -210;
    int maxWrap = 190;
    // init
    public TurretSS(CombinedCRServo servos, DcMotorEx encoder, PIDController pid, double lastTurretPos) {
        // variables
        this.servos = servos;
        this.encoder = encoder;
        this.status = TurretS.ZERO;
        this.pid = pid;
        // init
        if (lastTurretPos != -999) turretCpos = lastTurretPos;
        else encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // methods
    public void update(Follower follower) {
        // status
        if (atTarget(0)) status = TurretS.ZERO;
        if (!atTarget() && target != 0) status = TurretS.ROTATING;
        if (atTarget() && status == TurretS.ROTATING) status = TurretS.AT_TARGET;
        // update
        this.follower = follower;
        // pose
        targetPos = redSide ? redPos : bluePos;
        // grab current pos
        turretCpos = (-encoder.getCurrentPosition() / (TPR * ratio)) * 360;
        // turret code
        double error = wrap(target);
        double power = pid.calculate(error);
        power = Math.max(-1, Math.min(1, power));
        servos.setPower(power);
    }
    public void align() {
        setTarget(alignAngle());
    }
    private double alignAngle() {
        double dx = targetPos.getX() - follower.getPose().getX();
        double dy = targetPos.getY() - follower.getPose().getY();
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = Math.toDegrees(follower.getHeading()) - angleToGoal;
        return redSide ?  turretAngle + offset : turretAngle - offset;
    }
    private boolean atTarget(double target) {
        return turretCpos >= target - tolerance && turretCpos <= target + tolerance;
    }
    public double wrap(double angle) {
        if (angle > maxWrap) angle -= 360;
        if (angle < minWrap) angle += 360;
        return angle;
    }
    // setters
    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }
    public void setTarget(double target) {
        this.target = target;
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void setWrapAngles(int min, int max) {
        minWrap = min;
        maxWrap = max;
    }
    public void updatePID(PIDController pid) {
        this.pid = pid;
    }
    public void setRedSide(boolean redSide) {
        this.redSide = redSide;
    }
    public void setPoses(Pose bluePos, Pose redPos) {
        this.bluePos = bluePos;
        this.redPos = redPos;
    }
    // getters
    public boolean atTarget() {
        return turretCpos >= target - tolerance && turretCpos <= target + tolerance;
    }
    public double getCurrentPos() {
        return turretCpos;
    }
    public double getTarget() {
        return target;
    }
    public PIDController getPid() {
        return pid;
    }
    public Pose getTargetPose() {
        return targetPos;
    }
    public int getTolerance() {
        return tolerance;
    }
    public TurretS getStatus() {
        return status;
    }
    // telemetry
    public String telemetry() {
        return("===== Turret Telemetry =====\n" +
                "-- Positions --\n" +
                "Turret current pos: " + getCurrentPos() + "\n" +
                "Turret target pos: " + getTarget() + "\n" +
                "-- PID Values --\n" +
                "P: " + pid.getP() + "\n" +
                "I: " + pid.getI() + "\n" +
                "D: " + pid.getD() + "\n" +
                "-- Values --\n" +
                "Turret redSide: " + redSide + "\n" +
                "Align Turret output: " + wrap(alignAngle()) + "\n" +
                "Offset: " + offset + "\n" +
                "Tolerance: " + getTolerance() + "\n" +
                "Turret raw Power" + servos.getPower() + "\n" +
                "Status: " + getStatus() + "\n" +
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
