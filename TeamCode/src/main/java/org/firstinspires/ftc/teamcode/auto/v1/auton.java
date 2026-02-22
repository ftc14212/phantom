package org.firstinspires.ftc.teamcode.auto.v1;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleOp.MainV1;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@Autonomous(name = "auto", group = ".ftc14212")
public class auton extends OpMode {
    TelemetryM telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    ElapsedTime loopTime;
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
    public static double shooterVelo = 0;
    private boolean tReset = false;
    private boolean tReset2 = false;
    public boolean SHOOTER_READY = false;
    public static double shooterOffset = -18;
    public static boolean redSide = false;
    public static double blueShooter = 43;
    public static double redShooter = -49;
    public static boolean shooterOn = true;
    public static int intakeWait = 500;
    public static int gateWait = 2000;
    public static int shootWait = 1300;
    public static double turretPOffset = -0.0002;
    boolean ran2 = false;
    public static Pose target;
    public double distShooter;
    boolean ran = false;
    public static boolean indexerOn = true;
    public static int firstShotDelay = 600;
    private ElapsedTime tResetT;
    public static double speed = 1;
    public static boolean debugMode = true;
    public double turretCpos;
    public static double turretOffset = -10;
    public static boolean turretOn = true;
    public static boolean skipToGate = false;
    DigitalChannel beams;
    private final Prompter prompter = new Prompter(this);
    private MainV1E.Alliance alliance = MainV1E.Alliance.RED;
    private MainV1E.StartPos startPos = MainV1E.StartPos.FAR;
    TimerEx matchTime = new TimerEx(30); // 30 second autonomous
    private int pathState;
    // close
    private PathChain scoreClose, intakeClose, intakeMid, intakeFar, intakeGate, park;
    // blue close
    public static final Pose startPoseBC = new Pose(18, 119, Math.toRadians(144));
    public static final Pose shootClosePoseBC = new Pose(55.88, 84.65, Math.toRadians(180));
    public static final Pose intakeClosePoseBC = new Pose(20, 79, Math.toRadians(180));
    public static final Pose intakeCloseControlPoseBC = new Pose(50, 74, Math.toRadians(180));
    public static final Pose intakeMidPoseBC = new Pose(19, 57, Math.toRadians(180));
    public static final Pose intakeMidControlPoseBC = new Pose(62, 52, Math.toRadians(180));
    public static final Pose intakeFarPoseBC = new Pose(14.5, 34, Math.toRadians(180));
    public static final Pose intakeFarControlPoseBC = new Pose(72, 24.6, Math.toRadians(180));
    public static final Pose intakeGatePoseBC = new Pose(11, 60, Math.toRadians(145));
    public static final Pose intakeGateControlPoseBC = new Pose(51, 60, Math.toRadians(0));
    public static final Pose parkPoseBC = new Pose(26, 69.6, Math.toRadians(-90));
    // red close
    public static final Pose startPoseRC = startPoseBC.mirror();
    public static final Pose shootClosePoseRC = shootClosePoseBC.mirror();
    public static final Pose intakeClosePoseRC = intakeClosePoseBC.mirror();
    public static final Pose intakeCloseControlPoseRC = intakeCloseControlPoseBC.mirror();
    public static final Pose intakeMidPoseRC = intakeMidPoseBC.mirror();
    public static final Pose intakeMidControlPoseRC = intakeMidControlPoseBC.mirror();
    public static final Pose intakeFarPoseRC = intakeFarPoseBC.mirror();
    public static final Pose intakeFarControlPoseRC = intakeFarControlPoseBC.mirror();
    public static final Pose intakeGatePoseRC = new Pose(133, 58, Math.toRadians(40));
    public static final Pose intakeGateControlPoseRC = intakeGateControlPoseBC.mirror();
    public static final Pose parkPoseRC = parkPoseBC.mirror();
    // far
    private PathChain intake1, scoreFar1, grabBalls, scoreFar2, intake2, lever, scoreClose1, intake3, scoreClose2;
    // blue far
    public static final Pose startPoseBF = new Pose(56.5, 8.39, Math.toRadians(180));
    public static final Pose intakeStart1PoseBF = new Pose(46, 31, Math.toRadians(180));
    public static final Pose intakeEnd1PoseBF = new Pose(19.5, 34, Math.toRadians(180));
    public static final Pose shootFarPoseBF = new Pose(55.5, 13.36, Math.toRadians(180));
    public static final Pose grabBallsStartBF = new Pose(10, 44, Math.toRadians(-90));
    public static final Pose grabBallsEndBF = new Pose(7.7, 30, Math.toRadians(-90));
    public static final Pose intakeStart2PoseBF = new Pose(48.5, 51.5, Math.toRadians(180));
    public static final Pose intakeEnd2PoseBF = new Pose(19, 57, Math.toRadians(180));
    public static final Pose leverEndBF = new Pose(20.7, 69.2, Math.toRadians(180));
    public static final Pose leverControlBF = new Pose(38.7, 61.6, Math.toRadians(180));
    public static final Pose shootClosePoseBF = new Pose(55.88, 84.65, Math.toRadians(180));
    public static final Pose shootCloseControlBF = new Pose(51.37, 63.54, Math.toRadians(180));
    public static final Pose intakeEnd3PoseBF = new Pose(25, 82, Math.toRadians(180));
    public static final Pose parkPoseBF = new Pose(51, 73.5, Math.toRadians(180));
    // red far
    public static final Pose startPoseRF = startPoseBF.mirror();
    public static final Pose intakeStart1PoseRF = intakeStart1PoseBF.mirror();
    public static final Pose intakeEnd1PoseRF = intakeEnd1PoseBF.mirror();
    public static final Pose shootFarPoseRF = shootFarPoseBF.mirror();
    public static final Pose grabBallsStartRF = grabBallsStartBF.mirror();
    public static final Pose grabBallsEndRF = grabBallsEndBF.mirror();
    public static final Pose intakeStart2PoseRF = intakeStart2PoseBF.mirror();
    public static final Pose intakeEnd2PoseRF = intakeEnd2PoseBF.mirror();
    public static final Pose leverEndRF = leverEndBF.mirror();
    public static final Pose leverControlRF = leverControlBF.mirror();
    public static final Pose shootClosePoseRF = shootClosePoseBF.mirror();
    public static final Pose shootCloseControlRF = shootCloseControlBF.mirror();
    public static final Pose intakeEnd3PoseRF = intakeEnd3PoseBF.mirror();
    public static final Pose parkPoseRF = parkPoseBF.mirror();
    /* preload lines close */
    boolean shootCloseStarted = false;
    boolean intakeCloseStarted = false;
    boolean intakeMidStarted = false;
    boolean intakeFarStarted = false;
    boolean intakeGateStarted = false;
    boolean intakedClose = false;
    boolean intakedMid = false;
    boolean intakedFar = false;
    boolean intakedGate = false;
    boolean shoot = false;
    boolean shotFirst = true;
    /* preload lines far */
    boolean intake1Started = false;
    boolean shootFar1Started = false;
    boolean grabBallsStarted = false;
    boolean shootFar2Started = false;
    boolean intake2Started = false;
    boolean leverStarted = false;
    boolean scoreClose1Started = false;
    boolean intake3Started = false;
    boolean shootClose2Started = false;
    // idk
    boolean reached = false;
    boolean shootFar = false;
    boolean reached2 = false;
    public void buildPaths() {
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.BLUE) buildBlueClose();
            if (alliance == MainV1E.Alliance.RED) buildRedClose();
        }
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.BLUE) buildBlueFar();
            if (alliance == MainV1E.Alliance.RED) buildRedFar();
        }
    }

    private void buildBlueClose() {
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), shootClosePoseBC))
                .setConstantHeadingInterpolation(shootClosePoseBC.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeCloseControlPoseBC,
                        intakeClosePoseBC
                ))
                .setConstantHeadingInterpolation(intakeClosePoseBC.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeMidControlPoseBC,
                        intakeMidPoseBC
                ))
                .setConstantHeadingInterpolation(intakeMidPoseBC.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeFarControlPoseBC,
                        intakeFarPoseBC
                ))
                .setConstantHeadingInterpolation(intakeFarPoseBC.getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeGateControlPoseBC,
                        intakeGatePoseBC
                ))
                .setConstantHeadingInterpolation(intakeGatePoseBC.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), parkPoseBC))
                .setConstantHeadingInterpolation(parkPoseBC.getHeading())
                .build();
    }
    private void buildRedClose() {
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), shootClosePoseRC))
                .setConstantHeadingInterpolation(shootClosePoseRC.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeCloseControlPoseRC,
                        intakeClosePoseRC
                ))
                .setConstantHeadingInterpolation(intakeClosePoseRC.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeMidControlPoseRC,
                        intakeMidPoseRC
                ))
                .setConstantHeadingInterpolation(intakeMidPoseRC.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeFarControlPoseRC,
                        intakeFarPoseRC
                ))
                .setConstantHeadingInterpolation(intakeFarPoseRC.getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        intakeGateControlPoseRC,
                        intakeGatePoseRC
                ))
                .setConstantHeadingInterpolation(intakeGatePoseRC.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), parkPoseRC))
                .setConstantHeadingInterpolation(parkPoseRC.getHeading())
                .build();
    }
    private void buildBlueFar() {
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBF, intakeStart1PoseBF))
                .setConstantHeadingInterpolation(intakeStart1PoseBF.getHeading())
                .addPath(new BezierLine(intakeStart1PoseBF, intakeEnd1PoseBF))
                .setConstantHeadingInterpolation(intakeEnd1PoseBF.getHeading())
                .build();
        scoreFar1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1PoseBF, shootFarPoseBF))
                .setConstantHeadingInterpolation(shootFarPoseBF.getHeading())
                .build();
        grabBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPoseBF, grabBallsStartBF))
                .setConstantHeadingInterpolation(grabBallsStartBF.getHeading())
                .addPath(new BezierLine(grabBallsStartBF, grabBallsEndBF))
                .setConstantHeadingInterpolation(grabBallsEndBF.getHeading())
                .build();
        scoreFar2 = follower.pathBuilder()
                .addPath(new BezierLine(grabBallsEndBF, shootFarPoseBF))
                .setConstantHeadingInterpolation(shootFarPoseBF.getHeading())
                .build();
        intake2 = follower.pathBuilder()
                // .addPath(new BezierLine(shootFarPose, intakeStart2Pose))
                .addPath(new BezierLine(shootFarPoseBF, intakeStart2PoseBF))
                .setConstantHeadingInterpolation(intakeStart2PoseBF.getHeading())
                .addPath(new BezierLine(intakeStart2PoseBF, intakeEnd2PoseBF))
                .setConstantHeadingInterpolation(intakeEnd2PoseBF.getHeading())
                .build();
        lever = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeEnd2PoseBF,
                        leverControlBF,
                        leverEndBF
                ))
                .setConstantHeadingInterpolation(leverEndBF.getHeading())
                .build();
        scoreClose1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leverEndBF,
                        shootCloseControlBF,
                        shootClosePoseBF
                ))
                .setConstantHeadingInterpolation(shootClosePoseBF.getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePoseBF, intakeEnd3PoseBF))
                .setConstantHeadingInterpolation(intakeEnd3PoseBF.getHeading())
                .build();
        scoreClose2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd3PoseBF, shootClosePoseBF))
                .setConstantHeadingInterpolation(shootClosePoseBF.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePoseBF, parkPoseBF))
                .setConstantHeadingInterpolation(parkPoseBF.getHeading())
                .build();
    }
    private void buildRedFar() {
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRF, intakeStart1PoseRF))
                .setConstantHeadingInterpolation(intakeStart1PoseRF.getHeading())
                .addPath(new BezierLine(intakeStart1PoseRF, intakeEnd1PoseRF))
                .setConstantHeadingInterpolation(intakeEnd1PoseRF.getHeading())
                .build();
        scoreFar1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1PoseRF, shootFarPoseRF))
                .setConstantHeadingInterpolation(shootFarPoseRF.getHeading())
                .build();
        grabBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPoseRF, grabBallsStartRF))
                .setConstantHeadingInterpolation(grabBallsStartRF.getHeading())
                .addPath(new BezierLine(grabBallsStartRF, grabBallsEndRF))
                .setConstantHeadingInterpolation(grabBallsEndRF.getHeading())
                .build();
        scoreFar2 = follower.pathBuilder()
                .addPath(new BezierLine(grabBallsEndRF, shootFarPoseRF))
                .setConstantHeadingInterpolation(shootFarPoseRF.getHeading())
                .build();
        intake2 = follower.pathBuilder()
                // .addPath(new BezierLine(shootFarPose, intakeStart2Pose))
                .addPath(new BezierLine(shootFarPoseRF, intakeStart2PoseRF))
                .setConstantHeadingInterpolation(intakeStart2PoseRF.getHeading())
                .addPath(new BezierLine(intakeStart2PoseRF, intakeEnd2PoseRF))
                .setConstantHeadingInterpolation(intakeEnd2PoseRF.getHeading())
                .build();
        lever = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeEnd2PoseRF,
                        leverControlRF,
                        leverEndRF
                ))
                .setConstantHeadingInterpolation(leverEndRF.getHeading())
                .build();
        scoreClose1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leverEndRF,
                        shootCloseControlRF,
                        shootClosePoseRF
                ))
                .setConstantHeadingInterpolation(shootClosePoseRF.getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePoseRF, intakeEnd3PoseRF))
                .setConstantHeadingInterpolation(intakeEnd3PoseRF.getHeading())
                .build();
        scoreClose2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd3PoseRF, shootClosePoseRF))
                .setConstantHeadingInterpolation(shootClosePoseRF.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePoseRF, parkPoseRF))
                .setConstantHeadingInterpolation(parkPoseRF.getHeading())
                .build();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .onComplete(this::onPromptsComplete);
        MainV1E.lastAutoPos = null;
        telemetryM = new TelemetryM(telemetry, debugMode);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        tResetT = new ElapsedTime();
        timer = new ElapsedTime();
        opmodeTimer.resetTimer();
        loopTime = new ElapsedTime();
        loopTime.reset();
        follower = Constants.createFollower(hardwareMap);
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
    }

    public void onPromptsComplete() {
        alliance = prompter.get("alliance");
        startPos = prompter.get("start_pos");
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(startPoseRF);
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(startPoseBF);
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(startPoseRC);
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(startPoseBC);
        }
        redSide = alliance == MainV1E.Alliance.RED;
        intakedGate = redSide;
        MainV1.redSide = alliance == MainV1E.Alliance.RED;
        buildPaths();
        telemetryM.addLine("BLITZ Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.addData(true, "Alliance", alliance);
        telemetryM.addData(true, "Starting pos", startPos);
        telemetryM.update();
    }

    public void closeStates() {
        switch (pathState) {
            case 0:
                if (!shootCloseStarted) {
                    shoot = true;
                    reached = false;
                    intakeGateStarted = false;
                    follower.followPath(scoreClose, true);
                    shootCloseStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(shootClosePoseRC, 2, 2)) reached2 = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(shootClosePoseBC, 2, 2)) reached2 = true;
                if (reached2 && shootCloseStarted) {
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            timer.reset();
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        if (!shotFirst) {
                            if (timer.milliseconds() > firstShotDelay) {
                                FEED();
                                shotFirst = true;
                            }
                        } else FEED();
                        ledCpos = 1;
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        shoot = false;
                        if (skipToGate) setPathState(4);
                        else if (!intakedMid) setPathState(2);
                        else if (!intakedGate) setPathState(4);
                        else if (!intakedClose) setPathState(1);
                        else if (!intakedFar) setPathState(3);
                        else if (matchTime.isLessThan(3)) setPathState(5);
                    }
                }
                break;
            case 1:
                if (!intakeCloseStarted) {
                    reached2 = false;
                    speed = 1;
                    INTAKE();
                    follower.followPath(intakeClose, true);
                    shootCloseStarted = false;
                    intakedClose = true;
                    intakeCloseStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(intakeClosePoseRC, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(intakeClosePoseBC, 2, 2)) reached = true;
                if (reached) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(0);
                    }
                }
                break;
            case 2:
                if (!intakeMidStarted) {
                    reached2 = false;
                    indexerCpos = 1;
                    INTAKE();
                    follower.followPath(intakeMid, true);
                    shootCloseStarted = false;
                    intakeMidStarted = true;
                }
                if (!follower.isBusy() && intakeMidStarted) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        intakedMid = true;
                        setPathState(0);
                    }
                }
                break;
            case 3:
                if (!intakeFarStarted) {
                    reached2 = false;
                    speed = 1;
                    INTAKE();
                    follower.followPath(intakeFar, true);
                    shootCloseStarted = false;
                    intakedFar = true;
                    intakeFarStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(intakeFarPoseRC, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(intakeFarPoseBC, 2, 2)) reached = true;
                if (reached) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(0);
                    }
                }
                break;
            case 4:
                if (!intakeGateStarted) {
                    reached2 = false;
                    speed = 1;
                    indexerCpos = 1;
                    INTAKE();
                    follower.followPath(intakeGate, true);
                    shootCloseStarted = false;
                    intakedGate = true;
                    intakeGateStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > gateWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(0);
                    }
                }
                if (matchTime.isLessThan(4)) setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;

        }
    }
    
    
    public void farStates() {
        switch (pathState) {
            case 0:
                if (!intake1Started) {
                    turretTpos = redSide ? 67 : -67;
                    shootFar = true;
                    if (shooterR.getVelocity() >= shooterVelo + 10) {
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
                    turretTpos = redSide ? 68 : -68;
                    shoot = true;
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
                    if (shooterR.getVelocity() >= shooterVelo + 10) {
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
                        setPathState(4);
                    }
                }
                break;
            case 2:
                if (!grabBallsStarted) {
                    speed = 0.7;
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
                    turretTpos = redSide ? 68 : -68;
                    shootFar = true;
                    if (shooterR.getVelocity() >= shooterVelo + 10) {
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
                    turretTpos = redSide ? 45 : -45;
                    shootFar = true;
                    if (shooterR.getVelocity() >= shooterVelo + 10) {
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
                    turretTpos = redSide ? 45 : -45;
                    shootFar = true;
                    if (shooterR.getVelocity() >= shooterVelo + 10) {
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
        shoot = false;
    }

    public double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return redSide ?  turretAngle + turretOffset : turretAngle - turretOffset;
    }
    // wrap code
    public double wrap(double angle) {
        if (angle > 190) angle -= 360;
        if (angle < -210) angle += 360;
        return angle;
    }
    public double getShooterVelo(double distShooter) {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(15, 900);
        lut.add(25, 910);
        lut.add(35, 945);
        lut.add(45, 960);
        lut.add(55, 1005);
        lut.add(65, 1030);
        lut.add(75, 1070);
        lut.add(85, 1110);
        lut.add(105, 1300);
        lut.add(109, 1260);
        lut.add(115, 1280);
        lut.add(125, 1320);
        lut.add(135, 1360);
        // finish
        lut.createLUT();
        return lut.get(Math.max(15.1, Math.min(124.9, distShooter)));
    }
    public double getHoodCpos(double distShooter) {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(15, 0.0);
        lut.add(25, 0.0);
        lut.add(35, 0.1);
        lut.add(45, 0.2);
        lut.add(55, 0.31);
        lut.add(65, 0.30);
        lut.add(75, 0.23);
        lut.add(85, 0.27);
        lut.add(105, 0.4);
        lut.add(109, 0.6);
        lut.add(115, 0.25);
        lut.add(125, 0.3);
        lut.add(135, 0.4);
        // finish
        lut.createLUT();
        return lut.get(Math.max(15.1, Math.min(124.9, distShooter)));
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
        Pose bluePos = new Pose(11, 137, Math.toRadians(blueShooter));
        Pose redPos = new Pose(133, 137, Math.toRadians(redShooter));
        target = redSide ? redPos : bluePos;
        turretCpos = (-indexer.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
        double turretOffsetXY = Math.atan(target.getY()/follower.getPose().getX());
        double turretOffset = (Math.toDegrees(follower.getHeading()) - Math.toDegrees(target.getHeading())) + turretOffsetXY;
        distShooter = redSide ? Math.sqrt(Math.pow((redPos.getX() - follower.getPose().getX()), 2) + Math.pow((redPos.getY() - follower.getPose().getY()), 2)) : Math.sqrt(Math.pow((bluePos.getX() - follower.getPose().getX()), 2) + Math.pow((bluePos.getY() - follower.getPose().getY()), 2));
        distShooter += shooterOffset;
        SHOOTER_READY = shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80;
        if (!beams.getState() && !indexerOn) indexerCpos = 0;
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        follower.setMaxPower(speed);
        if (startPos == MainV1E.StartPos.FAR) farStates();
        if (startPos == MainV1E.StartPos.CLOSE) closeStates();
        // servos
        pivot.setPosition(pivotCpos);
        hood.setPosition(hoodCpos);
        indexer.setPower(indexerCpos);
        led.setPosition(ledCpos);
        // shooter code
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
        shooterR.setVelocity(shooterVelo); // leader
        shooterL.setVelocity(shooterVelo); // follower
        // turret code
        if (shoot) {
            turretTpos = turretOn ? wrap(
                    alignTurret(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toDegrees(follower.getHeading()),
                            target
                    )
            ) : 0;
            if (shooterOn) shooterVelo = getShooterVelo(distShooter);
            hoodCpos = getHoodCpos(distShooter);
        }
        if (shootFar) {
            if (shooterOn) shooterVelo = getShooterVelo(distShooter);
            hoodCpos = getHoodCpos(distShooter);
        }
        double error = turretTpos - turretCpos;
        double power = -turretPID.calculate(0, error) + PIDTuneTurret.F;
        power = Math.max(-1, Math.min(1, power));
        turret.setPower(power);
        follower.update();
        MainV1E.lastAutoPos = follower.getPose();
        // telemetry
        telemetryM.addLine("Blitz Team 14212!");
        telemetryM.addData(true, "reached",reached);
        telemetryM.addData(true, "reached2",reached2);
        telemetryM.addData(true, "shooter velo",shooterR.getVelocity());
        telemetryM.addData(true, "loop times", loopTime);
        telemetryM.addData(true, "reached", reached);
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
        telemetryM.addData(true, "getShooterVelo", getShooterVelo(distShooter));
        telemetryM.addData(true, "getHoodCpos", getHoodCpos(distShooter));
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
        telemetryM.addData(true, "alignTurret", alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), target));
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
        prompter.run();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        matchTime.start();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        MainV1E.lastAutoPos = follower.getPose(); // COMMENT THIS IF TELEOP AFTER AUTO NO WORKY
        MainV1E.lastTurretPos = turretCpos;
    }
}