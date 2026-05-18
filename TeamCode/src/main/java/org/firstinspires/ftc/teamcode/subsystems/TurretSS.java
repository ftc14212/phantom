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
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.SOTM;
import org.firstinspires.ftc.teamcode.vars.Tune;

public class TurretSS extends SubsystemBase {
    // hardware
    private final CombinedCRServo servos;
    private final DcMotorEx encoder;
    private final SOTM sotm = new SOTM();
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
    private PIDController FAR;
    private PIDController CLOSE;
    private double kF;
    private double offset = 0;
    private final double TPR = PIDTuneTurret.TPR;
    private final double ratio = PIDTuneTurret.ratio;
    private int minWrap = -210;
    private int maxWrap = 190;
    // init
    public TurretSS(CombinedCRServo servos, DcMotorEx encoder, Tune.PIDF pidFar, Tune.PIDF pidClose, double lastTurretPos) {
        // variables
        this.servos = servos;
        this.encoder = encoder;
        this.status = TurretS.ZERO;
        this.FAR = new PIDController(Math.sqrt(pidFar.P), pidFar.I, pidFar.D);
        this.CLOSE = new PIDController(Math.sqrt(pidClose.P), pidClose.I, pidClose.D);
        this.kF = pidFar.F;
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
        // pose
        targetPos = redSide ? redPos : bluePos;
        // grab current pos
        currentAngle = (-encoder.getCurrentPosition() / (TPR * ratio)) * 360;
        // turret code
        double error = wrap(target - currentAngle);
        double pidF = FAR.calculate(target, currentAngle);
        double pidC = CLOSE.calculate(target, currentAngle);
        double ff = 0;
        if (Math.abs(error) > 10) ff = Math.signum(error) * kF;
        double power = Math.abs(error) <= tolerance ? pidC + ff : pidF + ff;
        power = Math.max(-1, Math.min(1, power));
        servos.setPower(power);
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
    public void updatePID(Tune.PIDF pidFar, Tune.PIDF pidClose) {
        this.FAR = new PIDController(Math.sqrt(pidFar.P), pidFar.I, pidFar.D);
        this.CLOSE = new PIDController(Math.sqrt(pidClose.P), pidClose.I, pidClose.D);
        this.kF = pidFar.F;
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
    public PIDController getPidFar() {
        return FAR;
    }
    public PIDController getPidClose() {
        return CLOSE;
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
                "-- PID Far Values --\n" +
                "P: " + FAR.getP() + "\n" +
                "I: " + FAR.getI() + "\n" +
                "D: " + FAR.getD() + "\n" +
                "F: " + kF + "\n" +
                "-- PID Close Values --\n" +
                "P: " + FAR.getP() + "\n" +
                "I: " + FAR.getI() + "\n" +
                "D: " + FAR.getD() + "\n" +
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
