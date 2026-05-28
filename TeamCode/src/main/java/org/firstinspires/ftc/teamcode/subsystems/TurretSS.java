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

import org.firstinspires.ftc.teamcode.subsystems.enums.TurretS;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDDualTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.SOTM;
import org.firstinspires.ftc.teamcode.utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.vars.Tune;

public class TurretSS extends SubsystemBase {
    // hardware
    private final CombinedCRServo servos;
    private final DcMotorEx encoder;
    private final SOTM sotm = new SOTM();
    private final TrapezoidalMotionProfile profile;
    // data
    private double currentAngle = 0;
    private TurretS status;
    private double target = 0;
    private Follower follower = null;
    private Pose redPos = new Pose();
    private Pose bluePos = new Pose();
    private Pose targetPos = bluePos;
    private boolean redSide = false;
    // config
    private int tolerance = 10;
    private final PIDController controller;
    private double kF;
    private double offset = 0;
    private final double TPR = PIDDualTuneTurret.TPR;
    private final double ratio = PIDDualTuneTurret.ratio;
    private int minWrap = -210;
    private int maxWrap = 190;
    public static double maxVel = 180;
    public static double maxAccel = 360;
    // init
    public TurretSS(CombinedCRServo servos, DcMotorEx encoder, Tune.PIDF pidf, double lastTurretPos) {
        // variables
        this.servos = servos;
        this.encoder = encoder;
        this.status = TurretS.ZERO;
        this.controller = new PIDController(pidf.P, pidf.I, pidf.D);
        this.kF = pidf.F;
        this.profile = new TrapezoidalMotionProfile(maxVel, maxAccel);
        profile.reset(0);
        // init
        if (lastTurretPos != -999) currentAngle = lastTurretPos;
        else encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // methods
    public void update(Follower follower) {
        // status
        if (atTarget(0)) status = TurretS.ZERO;
        else if (atTarget()) status = TurretS.AT_TARGET;
        else status = TurretS.ROTATING;
        // update
        this.follower = follower;
        sotm.updateVelocity(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());
        profile.setConstraints(maxVel, maxAccel);
        // pose
        targetPos = redSide ? redPos : bluePos;
        // grab current pos
        currentAngle = (encoder.getCurrentPosition() / (TPR * ratio)) * 360;
        // turret code
        double profiledTarget = profile.update(wrap(target));
        double pid = controller.calculate(currentAngle, profiledTarget);
        double error = wrap(target - currentAngle);
        double ff = 0;
        if (Math.abs(error) > 2) ff = Math.signum(error) * kF;
        double rawPower = pid + ff;
        rawPower = Math.max(-1, Math.min(1, rawPower));
        // apply power
        servos.setPower(rawPower);
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
    private double sotmAngle() {
        return sotm.computeTurretAngle(follower.getPose(), targetPos);
    }
    private boolean atTarget(double target) {
        return Math.abs(wrap(target - currentAngle)) <= tolerance;
    }
    public double wrap(double angle) {
        while (angle > maxWrap) angle -= 360;
        while (angle < minWrap) angle += 360;
        return angle;
    }
    // setters
    public void align() {
        setTarget(alignAngle());
    }
    public void sotm() {
        setTarget(sotmAngle());
    }
    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }
    public void setTarget(double target) {
        this.target = wrap(target);
    }
    public void reset() {
        setTarget(0);
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void setWrapAngles(int min, int max) {
        minWrap = min;
        maxWrap = max;
    }
    public void updatePID(Tune.PIDF pidf) {
        controller.setPID(pidf.P, pidf.I, pidf.D);
        this.kF = pidf.F;
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
        return Math.abs(wrap(target - currentAngle)) <= tolerance;
    }
    public double getCurrentPos() {
        return currentAngle;
    }
    public double getTarget() {
        return target;
    }
    public PIDController getPid() {
        return controller;
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
                "P: " + controller.getP() + "\n" +
                "I: " + controller.getI() + "\n" +
                "D: " + controller.getD() + "\n" +
                "F: " + kF + "\n" +
                "-- Values --\n" +
                "Turret redSide: " + redSide + "\n" +
                "Offset: " + offset + "\n" +
                "Tolerance: " + getTolerance() + "\n" +
                "Status: " + getStatus() + "\n" +
                "-- Outputs --\n" +
                "Align Turret output: " + wrap(alignAngle()) + "\n" +
                "SOTM output: " + wrap(sotmAngle()) + "\n" +
                "Turret raw Power" + servos.getPower() + "\n" +
                "Profile velocity" + profile.getVelocity() + "\n" +
                "Profile Targer" + profile.update(target) + "\n" +
                "-- Poses --\n" +
                "Follower:" + "\n" +
                "X: " + follower.getPose().getX() + "\n" +
                "Y: " + follower.getPose().getY() + "\n" +
                "heading: " + Math.toDegrees(follower.getHeading()) + "\n" +
                "\nSOTM pos:" + "\n" +
                "X: " + sotm.predictRobotPose(follower.getPose()).getX() + "\n" +
                "Y: " + sotm.predictRobotPose(follower.getPose()).getY() + "\n" +
                "heading: " + Math.toDegrees(sotm.predictRobotPose(follower.getPose()).getHeading()) + "\n" +
                "\nTarget Goal pos:" + "\n" +
                "X: " + targetPos.getX() + "\n" +
                "Y: " + targetPos.getY() + "\n" +
                "heading: " + Math.toDegrees(targetPos.getHeading()) + "\n");
    }
}
