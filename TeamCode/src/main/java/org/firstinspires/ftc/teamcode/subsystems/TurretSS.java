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
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.SOTM;
import org.firstinspires.ftc.teamcode.utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.vars.Tune;

public class TurretSS extends SubsystemBase {
    // hardware
    private final CombinedCRServo servos;
    private final CombinedServo servo;
    private DcMotorEx encoder;
    private final SOTM sotm = new SOTM();
    private TrapezoidalMotionProfile profile;
    // data
    private double currentAngle = 0;
    private TurretS status;
    private double target = 0;
    private Follower follower = null;
    private Pose redPos = new Pose();
    private Pose bluePos = new Pose();
    private Pose targetPos = bluePos;
    private boolean redSide = false;
    private double profiledTarget;
    private boolean positional = false;
    // config
    private int tolerance = 10;
    private PIDController controller;
    private double kF;
    private double offset = 0;
    private final double TPR = PIDTuneTurret.TPR;
    private final double ratio = PIDTuneTurret.ratio;
    private int minWrap = -210;
    private int maxWrap = 190;
    public static double maxVel = 180;
    public static double maxAccel = 360;
    public static double positionalRatio = 460.8;
    // init
    public TurretSS(CombinedCRServo servos, DcMotorEx encoder, Tune.PIDF pidf, double lastTurretPos) {
        // variables
        this.servos = servos;
        this.servo = null;
        this.positional = false;
        init(encoder, pidf, lastTurretPos);
    }
    public TurretSS(CombinedServo servos, DcMotorEx encoder, Tune.PIDF pidf, double lastTurretPos) {
        // variables
        this.servos = null;
        this.servo = servos;
        this.positional = true;
        servo.setPosition(0);
        init(encoder, pidf, lastTurretPos);
    }
    private void init(DcMotorEx encoder, Tune.PIDF pidf, double lastTurretPos) {
        this.encoder = encoder;
        this.status = TurretS.ZERO;
        this.controller = new PIDController(pidf.P, pidf.I, pidf.D);
        this.kF = pidf.F;
        this.profile = new TrapezoidalMotionProfile(maxVel, maxAccel);
        profile.reset(0);
        if (lastTurretPos != -999) {
            currentAngle = lastTurretPos;
            profile.reset(lastTurretPos);
        } else {
            profile.reset(0);
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
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
        profiledTarget = profile.update(target);
        if(!positional) {
            double pid = controller.calculate(currentAngle, profiledTarget);
            // feedforward
            double error =  wrap(profiledTarget - currentAngle);
            double ff = 0;
            if (Math.abs(error) > 2) ff = Math.signum(error) * kF;
            double rawPower = pid + ff;
            rawPower = Math.max(-1, Math.min(1, rawPower));
            // apply power
            servos.setPower(rawPower);
        } else {
            if (servo.getPosition() == 0) servo.setPosition(Math.round(degreeToPos(target) * 10.0) / 10.0);
            else servo.setPosition(degreeToPos(target));
        }
    }
    private double alignAngle() {
        double dx = targetPos.getX() - follower.getPose().getX();
        double dy = targetPos.getY() - follower.getPose().getY();
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - Math.toDegrees(follower.getHeading());
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
    public double degreeToPos(double target) {
        double pos = 0.5 + target / positionalRatio;
        return Math.max(0.0, Math.min(1.0, pos));
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
        return "===== Turret Telemetry =====\n" +
                "-- Positions --\n" +
                "Turret current pos: " + getCurrentPos() + "\n" +
                "Turret target pos: " + getTarget() + "\n" +
                (positional ? ("Turret servo pos: " + servo.getPosition() + "\n") : "") +
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
                (positional ? "" : "Turret raw Power: " + servos.getPower() + "\n") +
                "Profile velocity: " + profile.getVelocity() + "\n" +
                "Profile Target: " + profiledTarget + "\n" +
                "-- Poses --\n" +
                "Follower:\n" +
                "X: " + follower.getPose().getX() + "\n" +
                "Y: " + follower.getPose().getY() + "\n" +
                "heading: " + Math.toDegrees(follower.getHeading()) + "\n" +
                "\nSOTM pos:\n" +
                "X: " + sotm.predictRobotPose(follower.getPose()).getX() + "\n" +
                "Y: " + sotm.predictRobotPose(follower.getPose()).getY() + "\n" +
                "heading: " + Math.toDegrees(sotm.predictRobotPose(follower.getPose()).getHeading()) + "\n" +
                "\nTarget Goal pos:\n" +
                "X: " + targetPos.getX() + "\n" +
                "Y: " + targetPos.getY() + "\n" +
                "heading: " + Math.toDegrees(targetPos.getHeading()) + "\n";
    }
}
