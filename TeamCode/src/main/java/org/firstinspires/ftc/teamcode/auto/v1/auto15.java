package org.firstinspires.ftc.teamcode.auto.v1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Disabled
@Configurable
@Autonomous(name = "15 baller auto", group = ".ftc14212")
public class auto15 extends OpMode {
    TelemetryM telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer;
    PIDController shooterPID;
    PIDController turretPID;
    // gamepads
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    // motors
    CombinedCRServo turret; // 223 rpm
    CachingDcMotorEx shooterL; // 6000 rpm
    CachingDcMotorEx shooterR; // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm
    // servos
    private static CachingServo pivot; // 1x axon max
    private static CachingServo hood; // 1x axon mini
    private static CachingServo led; // 2x gobilda led lights RGB
    private static CachingDcMotorEx indexer; // 2x axon minis
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    public static double turretTpos = 0;
    public static int shooterVelo = 0;
    private boolean tReset = false;
    private boolean tReset2 = false;
    public boolean SHOOTER_READY = false;
    public static boolean redSide = false;
    public static double blueShooter = 43;
    public static double redShooter = -49;
    public static boolean shooterOn = true;
    public static int intakeWait = 500;
    public static int shootWait = 1300;
    public static double turretPOffset = -0.0002;
    boolean ran2 = false;
    public static Pose target;
    public double distShooter;
    boolean ran = false;
    public static boolean indexerOn = true;
    private ElapsedTime tResetT;
    public static double speed = 1;
    public static boolean debugMode = true;
    DigitalChannel beams;
    private int pathState;
    public static final Pose startPose = new Pose(56.5, 8.39, Math.toRadians(180));
    public static final Pose intakeStart1Pose = new Pose(46, 31, Math.toRadians(180));
    public static final Pose intakeEnd1Pose = new Pose(19.5, 34, Math.toRadians(180));
    public static final Pose shootFarPose = new Pose(55.5, 13.36, Math.toRadians(180));
    public static final Pose grabBallsStart = new Pose(17.4, 11.2, Math.toRadians(-165));
    public static final Pose grabBallsEnd = new Pose(11.5, 9.7, Math.toRadians(-180));
    public static final Pose intakeStart2Pose = new Pose(48.5, 51.5, Math.toRadians(180));
    public static final Pose intakeEnd2Pose = new Pose(19, 57, Math.toRadians(180));
    public static final Pose leverEnd = new Pose(20.7, 69.2, Math.toRadians(180));
    public static final Pose leverControl = new Pose(38.7, 61.6, Math.toRadians(180));
    public static final Pose shootClosePose = new Pose(55.88, 84.65, Math.toRadians(180));
    public static final Pose shootCloseControl = new Pose(51.37, 63.54, Math.toRadians(180));
    public static final Pose intakeEnd3Pose = new Pose(25, 82, Math.toRadians(180));
    public static final Pose parkPose = new Pose(51, 73.5, Math.toRadians(180));
    private PathChain intake1, scoreFar1, grabBalls, scoreFar2, intake2, lever, scoreClose1, intake3, scoreClose2, park;
    /* preload lines */
    boolean intake1Started = false;
    boolean shootFar1Started = false;
    boolean grabBallsStarted = false;

    boolean shootFar2Started = false;
    boolean intake2Started = false;
    boolean leverStarted = false;

    boolean scoreClose1Started = false;
    boolean intake3Started = false;
    boolean shootClose2Started = false;
    boolean parkStarted = false;
    public double turretCpos;

    public void buildPaths() {
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakeStart1Pose))
                .setConstantHeadingInterpolation(intakeStart1Pose.getHeading())
                .addPath(new BezierLine(intakeStart1Pose, intakeEnd1Pose))
                .setConstantHeadingInterpolation(intakeEnd1Pose.getHeading())
                .build();
        scoreFar1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1Pose, shootFarPose))
                .setConstantHeadingInterpolation(shootFarPose.getHeading())
                .build();
        grabBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, grabBallsStart))
                .setConstantHeadingInterpolation(grabBallsStart.getHeading())
                .addPath(new BezierLine(grabBallsStart, grabBallsEnd))
                .setConstantHeadingInterpolation(grabBallsEnd.getHeading())
                .build();
        scoreFar2 = follower.pathBuilder()
                .addPath(new BezierLine(grabBallsEnd, shootFarPose))
                .setConstantHeadingInterpolation(shootFarPose.getHeading())
                .build();
        intake2 = follower.pathBuilder()
                // .addPath(new BezierLine(shootFarPose, intakeStart2Pose))
                .addPath(new BezierLine(shootFarPose, intakeStart2Pose))
                .setConstantHeadingInterpolation(intakeStart2Pose.getHeading())
                .addPath(new BezierLine(intakeStart2Pose, intakeEnd2Pose))
                .setConstantHeadingInterpolation(intakeEnd2Pose.getHeading())
                .build();
        lever = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeEnd2Pose,
                        leverControl,
                        leverEnd
                ))
                .setConstantHeadingInterpolation(leverEnd.getHeading())
                .build();
        scoreClose1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leverEnd,
                        shootCloseControl,
                        shootClosePose
                ))
                .setConstantHeadingInterpolation(shootClosePose.getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, intakeEnd3Pose))
                .setConstantHeadingInterpolation(intakeEnd3Pose.getHeading())
                .build();
        scoreClose2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd3Pose, shootClosePose))
                .setConstantHeadingInterpolation(shootClosePose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        telemetryM = new TelemetryM(telemetry, debugMode);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        tResetT = new ElapsedTime();
        timer = new ElapsedTime();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        // hi
        shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P + turretPOffset), PIDTuneTurret.I, PIDTuneTurret.D);
        beams = hardwareMap.get(DigitalChannel.class, "bb");
        // gamepads
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        // motors
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm
        indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        // servos
        pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        // directions
        beams.setMode(DigitalChannel.Mode.INPUT);
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        // reset pos
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // limits
        pivot.scaleRange(0, 0.4);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
        indexerCpos = 0;
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // telemetry
        telemetryM.addLine("BLITZ Team 14212!");
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!intake1Started) {
                    turretTpos = -68;
                    shooterVelo = 1280;
                    hoodCpos = 0.5;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        OUTTAKE();
                        speed = 1;
                        follower.followPath(intake1, true);
                        intake1Started = true;
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        INTAKE();
                        speed = 1;
                        follower.followPath(intake1, true);
                        intake1Started = true;
                    }
                }
                if (!follower.isBusy() && intake1Started) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (!shootFar1Started) {
                    follower.followPath(scoreFar1, true);
                    shootFar1Started = true;
                }
                if (!follower.isBusy() && shootFar1Started) {
                    turretTpos = -68;
                    shooterVelo = 1280;
                    hoodCpos = 0.5;
                    if (!ran2) {
                        indexerCpos = -0.45;
                        ran2 = true;
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        OUTTAKE();
                        ran = false;
                        ran2 = false;
                        setPathState(4);
                    }
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!grabBallsStarted) {
                    speed = 0.65  ;
                    INTAKE();
                    follower.followPath(grabBalls, true);
                    grabBallsStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!shootFar2Started) {
                    follower.followPath(scoreFar2, true);
                    shootFar2Started = true;
                }
                if (!follower.isBusy() && shootFar2Started) {
                    turretTpos = -68;
                    shooterVelo = 1280;
                    hoodCpos = 0.5;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        OUTTAKE();
                        ran = false;
                        ran2 = false;
                        setPathState(4);
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!intake2Started) {
                    speed = 1;
                    indexerCpos = 1;
                    INTAKE();
                    follower.followPath(intake2, true);
                    intake2Started = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!leverStarted) {
                    follower.followPath(lever, true);
                    leverStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (!scoreClose1Started) {
                    follower.followPath(scoreClose1, true);
                    scoreClose1Started = true;
                }
                if (!follower.isBusy() && scoreClose1Started) {
                        turretTpos = -46;
                        shooterVelo = 960;
                        hoodCpos = 0.2;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        OUTTAKE();
                        ran = false;
                        ran2 = false;
                        setPathState(7);
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!intake3Started) {
                    speed = 1;
                    INTAKE();
                    follower.followPath(intake3, true);
                    intake3Started = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!shootClose2Started) {
                    follower.followPath(scoreClose2, true);
                    shootClose2Started = true;
                }
                if (!follower.isBusy() && shootClose2Started) {
                    turretTpos = -46;
                    shooterVelo = 960;
                    hoodCpos = 0.2;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        OUTTAKE();
                        ran = false;
                        ran2 = false;
                        setPathState(9);
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;

        }
    }

    public void INTAKE() {
        indexerOn = false;
        pivotCpos = 0.75;
        indexerCpos = 1;
        intake.setPower(1);
        shooterVelo = -65;
    }

    public void OUTTAKE() {
        indexerOn = true;
        pivotCpos = 0.75;
        indexerCpos = -1;
        intake.setPower(-1);
    }
    public void RESET_INTAKE() {
        pivotCpos = 0.45;
        indexerCpos = 0;
        intake.setPower(0);
        shooterVelo = 0;
        indexerOn = true;
    }
    public void FEED() {
        indexerOn = true;
        pivotCpos = 0.45;
        indexerCpos = 1;
        intake.setPower(1);
    }
    public void RESET_SHOOTER_TURRET() {
        tReset = true;
        tResetT.reset();
        turretTpos = 0;
        ledCpos = 0.611;
        shooterVelo = 0;
        hoodCpos = 0;
        indexerOn = true;
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        Pose bluePos = new Pose(114.9, 24.7, Math.toRadians(blueShooter));
        Pose redPos = new Pose(124.9, -62, Math.toRadians(redShooter));
        target = redSide ? redPos : bluePos;
        turretCpos = (-indexer.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
        double turretOffsetXY = Math.atan(target.getY()/follower.getPose().getX());
        double turretOffset = (Math.toDegrees(follower.getHeading()) - Math.toDegrees(target.getHeading())) + turretOffsetXY;
        distShooter = redSide ? Math.sqrt(Math.pow((redPos.getX() - follower.getPose().getX()), 2) + Math.pow((redPos.getY() - follower.getPose().getY()), 2)) : Math.sqrt(Math.pow((bluePos.getX() - follower.getPose().getX()), 2) + Math.pow((bluePos.getY() - follower.getPose().getY()), 2));
        SHOOTER_READY = shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80;
        if (!beams.getState() && !indexerOn) indexerCpos = 0;
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        follower.setMaxPower(speed);
        autonomousPathUpdate();
        // servos
        pivot.setPosition(pivotCpos);
        hood.setPosition(hoodCpos);
        indexer.setPower(indexerCpos);
        led.setPosition(ledCpos);
        // shooter code
        double sPower = shooterOn ? shooterPID.calculate(shooterR.getVelocity(), shooterVelo) + shooterVelo : 0;
        shooterR.setVelocity(sPower); // leader
        shooterL.setVelocity(sPower); // follower
        // turret code
        double error = turretTpos - turretCpos;
        double power = -turretPID.calculate(0, error) + PIDTuneTurret.F;
        power = Math.max(-1, Math.min(1, power));
        turret.setPower(power);
        follower.update();
        // telemetry
        telemetryM.addLine("Blitz Team 14212!");
        telemetryM.addData(true, "pivot", pivot.getPosition());
        telemetryM.addData(true, "hood", hood.getPosition());
        telemetryM.addData(true, "indexer", indexer.getPower());
        telemetryM.addData(true, "led", led.getPosition());
        telemetryM.addData(true, "turret", turretCpos);
        telemetryM.addData(true, "turretOffset", turretOffset);
        telemetryM.addData(true, "bluePos", Math.toDegrees(bluePos.getHeading()));
        telemetryM.addData(true, "redPos", Math.toDegrees(redPos.getHeading()));
        telemetryM.addData(true, "turretPower", turret.getPower());
        telemetryM.addData(true, "tReset", tReset);
        telemetryM.addData(true, "tReset2", tReset2);
        telemetryM.addData(true,"turret error", Math.abs(turretTpos - turretCpos));
        telemetryM.addData(true, "PIDF", "P: " + PIDTuneTurret.P + " I: " + PIDTuneTurret.I + " D: " + PIDTuneTurret.D + " F: " + PIDTuneTurret.F);
        telemetryM.addData(true, "turretTpos", turretTpos);
        telemetryM.addData(true, "shooterR Current", shooterR.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryM.addData(true, "indexerOn", indexerOn);
        telemetryM.addData(true, "distShooter", distShooter);
        // telemetryM.addData(true, "getShooterVelo", getShooterVelo(distShooter));
        // telemetryM.addData(true, "getHoodCpos", getHoodCpos(distShooter));
        telemetryM.addData(true, "redSide", redSide);
        telemetryM.addData(true, "\nFollower:\nX:", follower.getPose().getX());
        telemetryM.addData(true, "Y:", follower.getPose().getY());
        telemetryM.addData(true, "heading:", Math.toDegrees(follower.getHeading()));
        telemetryM.addData(true, "\nredPos:\nX:", redPos.getX());
        telemetryM.addData(true, "Y:", redPos.getY());
        telemetryM.addData(true, "heading:", Math.toDegrees(redPos.getHeading()));
        telemetryM.addData(true, "\nbluePos:\nX:", bluePos.getX());
        telemetryM.addData(true, "Y:", bluePos.getY());
        telemetryM.addData(true, "heading:", Math.toDegrees(bluePos.getHeading()));
        telemetryM.addData(true, "\nturret offset XY", turretOffsetXY);
        // telemetryM.addData(true, "alignTurret", alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), target));
        telemetryM.addData(true, "path state", pathState);
        telemetryM.addData(true, "x", follower.getPose().getX());
        telemetryM.addData(true, "y", follower.getPose().getY());
        telemetryM.addData(true, "heading", follower.getPose().getHeading());
        telemetryM.update();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        MainV1E.lastAutoPos = follower.getPose();
        MainV1E.lastTurretPos = turretCpos;
    }
}