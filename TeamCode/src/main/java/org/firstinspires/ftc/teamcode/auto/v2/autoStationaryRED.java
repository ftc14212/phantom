package org.firstinspires.ftc.teamcode.auto.v2;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV2;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@Autonomous(name = "auto Stationary RED", group = ".ftc14212")
public class autoStationaryRED extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int shootWait = 1300;
    private int pathState;
    private final Pose startPose = new Pose(126, 119, Math.toRadians(37));
    public static double pivotCpos = 0.45;
    private final Pose shootClose = new Pose(94, 85, Math.toRadians(48));
    private final Pose parkPose = new Pose(118, 69.6, Math.toRadians(-90));
    private PathChain score, park;
    boolean shootCloseStarted = false;
    boolean reached2 = false;
    boolean ran = false;
    boolean ran2 = false;
    private Timer timer;
    private Timer timer2;
    PIDController turretPID;
    PIDFCoefficients shooterPID;
    // subsystems
    TurretSS turretSS;
    ShooterSS shooterSS;
    // motors
    CachingDcMotorEx shooterL; // 6000 rpm
    CachingDcMotorEx shooterR; // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm --> 575 rpm
    CachingDcMotorEx indexer; // 1620 rpm --> 810 rpm
    // servos
    CachingServo pivot; // 1x axon max
    CachingServo stopper; // 1x axon mini
    CombinedServo hood; // 2x axon mini
    CachingServo led; // 2x gobilda led lights RGB
    CachingServo strips; // 4x gobilda strip RGB lights
    CombinedCRServo turret; // 2x axon mini

    public void buildPaths() {
        score = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootClose, parkPose))
                .setLinearHeadingInterpolation(shootClose.getHeading(), parkPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!shootCloseStarted) {
                    ran = false;
                    ran2 = false;
                    timer.resetTimer();
                    timer2.resetTimer();
                    shooterSS.stopBackSpin();
                    shooterSS.shooterOn(true);
                    follower.followPath(score, true);
                    shootCloseStarted = true;
                }
                if (follower.atPose(shootClose, 8, 8)) reached2 = true;
                if (reached2 && shootCloseStarted) {
                    if (shooterR.getVelocity() >= shooterSS.getTargetVelocity()) {
                        if (!ran2) {
                            timer2.resetTimer();
                            ran2 = true;
                        }
                        if (timer2.getElapsedTime() >= 800) {
                            if (!ran) {
                                timer.resetTimer();
                                indexer.setPower(1);
                                intake.setPower(1);
                                ran = true;
                            }
                        }
                    }
                    if ((ran && timer.getElapsedTime() >= shootWait)) {
                        RESET_SHOOTER_TURRET();
                        RESET_INTAKE();
                        ran = false;
                        ran2 = false;
                        setPathState(2);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    shooterSS.shooterOn(false);
                    RESET_SHOOTER_TURRET();
                    RESET_INTAKE();
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void RESET_INTAKE() {
        pivotCpos = 0.45;
        indexer.setPower(0);
        intake.setPower(0);
    }
    public void FEED() {
        pivotCpos = 0.45;
        indexer.setPower(1);
        intake.setPower(1);
    }
    public void RESET_SHOOTER_TURRET() {
        shooterSS.shooterOn(false);
        turretSS.turretOn(false); // i could so go for a hot coca right now
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        Pose bluePos = new Pose(9, 138, 135); // BYE
        Pose redPos = new Pose(138, 138, 45);
        turretPID.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(55,0,0,12);
        turretSS.updatePID(turretPID, PIDTuneTurret.F);
        turretSS.setTurretOffset(-8);
        shooterSS.updatePID(shooterPID); // woahhh
        shooterSS.setShooterOffset(-16);
        shooterSS.setPoses(bluePos, redPos); // woah
        turretSS.setPoses(bluePos, redPos);
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        pivot.setPosition(pivotCpos);
        // shooter code
        shooterSS.alignShooter();
        shooterSS.update(follower);
        // turret code
        turretSS.alignTurret();
        turretSS.update(follower);
        follower.update();
        MainV1E.lastAutoPos = follower.getPose();
        MainV1E.lastTurretPos = turretSS.getCurrentPos();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        timer = new Timer();
        timer2 = new Timer();
        // motors
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm --> 460 rpm
        indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        // servos
        pivot = new CachingServo(hardwareMap.get(Servo.class, "pivot")); // 1x axon max
        CachingServo hoodR = new CachingServo(hardwareMap.get(Servo.class, "hoodR")); // 1x axon mini
        CachingServo hoodL = new CachingServo(hardwareMap.get(Servo.class, "hoodL")); // 1x axon mini
        hood = new CombinedServo(hoodR, hoodL); // 2x axon minis
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon mini
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon mini
        turret = new CombinedCRServo(turret1, turret2); // 2x axon minis
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        strips = new CachingServo(hardwareMap.get(Servo.class, "strips")); // 4x gobilda strip RGB lights
        stopper = new CachingServo(hardwareMap.get(Servo.class, "stopper")); // 1x axon mini
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        indexer.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // subsystems
        shooterSS = new ShooterSS(shooterPID, shooterR, shooterL, hoodR, hoodL);
        shooterSS.setPoses(MainV2.getShooterLUT(), 15.1, 124.9, MainV2.getHoodLut(), 15.1, 124.9);
        shooterSS.update(follower);
        turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, indexer, PIDTuneTurret.TPR, PIDTuneTurret.ratio, 0, MainV1E.lastTurretPos);
        turretSS.setRedSide(true);
        shooterSS.setRedSide(true);
        turretSS.setWrapAngles(-180, 180);
        turretSS.update(follower);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
