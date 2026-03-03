package org.firstinspires.ftc.teamcode.auto.v2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.auto.v2.points.BC;
import org.firstinspires.ftc.teamcode.auto.v2.points.BF;
import org.firstinspires.ftc.teamcode.auto.v2.points.RC;
import org.firstinspires.ftc.teamcode.auto.v2.points.RF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV2;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@Autonomous(name = "auto", group = ".ftc14212")
public class auto extends OpMode {
    TelemetryM telemetryM;
    private Follower follower;
    // timers
    ElapsedTime loopTime;
    Timer gameTimer = new Timer();
    private Timer timer;
    private Timer timer2;
    private Timer shotTimer;
    // gamepads
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
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
    // pids
    PIDController turretPID;
    PIDFCoefficients shooterPID;
    // subsystems
    TurretSS turretSS;
    ShooterSS shooterSS;
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 0;
    public static double ledCpos = 0.667;
    public static double stripsCpos = 0.611;
    public static double turretTpos = 0;
    public static double shooterVelo = 0; // update servos r kissing
    public static double initGameStrips = 0.75;
    // misc
    private double wheelSpeed = 1;
    public static boolean turretOn = true;
    boolean indexerOn = true;
    public static double turretOffsetR = -3; // kabam
    public static double turretOffsetB = 0; // kabam
    public static double shooterOffset = -15;
    public static boolean debugMode = true;
    public static boolean redSide = false;
    public static int intakeWait = 1000;
    public static int humanWait = 2000;
    public static int gateWait = 2000;
    public static int shootWait = 1300;
    DigitalChannel beams;
    private final Prompter prompter = new Prompter(this);
    private MainV1E.Alliance alliance = MainV1E.Alliance.RED;
    private MainV1E.StartPos startPos = MainV1E.StartPos.FAR;
    TimerEx matchTime = new TimerEx(30); // 30 second autonomous
    private int pathState;
    boolean reached = false;
    boolean reached2 = false;
    public static boolean shooterOn = true;
    int shot = 0;
    boolean ran = false;
    boolean ran2 = false;
    boolean gate = false;
    boolean humanPlayer = false;
    // close
    private PathChain scoreClose, intakeClose, intakeMid, intakeFar, intakeGate, park;
    // close
    boolean shootCloseStarted = false;
    boolean intakeCloseStarted = false;
    boolean intakeMidStarted = false;
    boolean intakeFarStarted = false;
    boolean intakeGateStarted = false;
    boolean intakedClose = false;
    boolean intakedMid = false;
    boolean intakedFar = false;
    boolean intakedGate = false;
    // far
    private PathChain shootFar, leave, human;
    // far
    boolean shootStarted = false;
    boolean leaveStarted = false;
    boolean humanStarted = false;

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

    private void buildBlueFar() {
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BF.parkPose
                ))
                .setConstantHeadingInterpolation(BF.parkPose.getHeading())
                .build();
        human = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BF.humanPose
                ))
                .setConstantHeadingInterpolation(BF.humanPose.getHeading())
                .build();
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BF.startPose
                ))
                .setConstantHeadingInterpolation(BF.startPose.getHeading())
                .build();
    }
    private void buildRedFar() {
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RF.parkPose
                ))
                .setConstantHeadingInterpolation(RF.parkPose.getHeading())
                .build();
        human = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RF.humanPose
                ))
                .setConstantHeadingInterpolation(RF.humanPose.getHeading())
                .build();
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RF.startPose
                ))
                .setConstantHeadingInterpolation(RF.startPose.getHeading())
                .build();
    }
    private void buildBlueClose() {
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BC.shootClosePose
                ))
                .setConstantHeadingInterpolation(BC.shootClosePose.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BC.intakeClosePose
                ))
                .setConstantHeadingInterpolation(BC.intakeClosePose.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BC.shootClosePose,
                        BC.intakeMidControlPose,
                        BC.intakeMidPose
                ))
                .setConstantHeadingInterpolation(BC.intakeMidPose.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BC.shootClosePose,
                        BC.intakeFarControlPose,
                        BC.intakeFarPose
                ))
                .setConstantHeadingInterpolation(BC.intakeFarPose.getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BC.intakeGatePose
                ))
                .setConstantHeadingInterpolation(BC.intakeGatePose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        BC.parkPose
                ))
                .setConstantHeadingInterpolation(BC.parkPose.getHeading())
                .build();
    }
    private void buildRedClose() {
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RC.shootClosePose
                ))
                .setConstantHeadingInterpolation(RC.shootClosePose.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RC.intakeClosePose
                ))
                .setConstantHeadingInterpolation(RC.intakeClosePose.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        RC.shootClosePose,
                        RC.intakeMidControlPose,
                        RC.intakeMidPose
                ))
                .setConstantHeadingInterpolation(RC.intakeMidPose.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        RC.shootClosePose,
                        RC.intakeFarControlPose,
                        RC.intakeFarPose
                ))
                .setConstantHeadingInterpolation(RC.intakeFarPose.getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RC.intakeGatePose
                ))
                .setConstantHeadingInterpolation(RC.intakeGatePose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        RC.parkPose
                ))
                .setConstantHeadingInterpolation(RC.parkPose.getHeading())
                .build();
    }


    public void closeStates() {
        switch (pathState) {
            case 0:
                if (!shootCloseStarted) {
                    ran = false;
                    ran2 = false;
                    timer.resetTimer();
                    timer2.resetTimer();
                    shooterSS.stopBackSpin();
                    if (shooterOn) shooterSS.shooterOn(true);
                    if (turretOn) turretSS.turretOn(true);
                    ledCpos = 0.388;
                    intakeGateStarted = false;
                    reached = false;
                    follower.followPath(scoreClose, true);
                    shootCloseStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RC.shootClosePose, 8, 8)) reached2 = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BC.shootClosePose, 8, 8)) reached2 = true;
                if (reached2 && shootCloseStarted) {
                    if (shooterR.getVelocity() >= shooterSS.getTargetVelocity()) {
                        if (!ran2) {
                            timer2.resetTimer();
                            ran2 = true;
                        }
                        if (timer2.getElapsedTimeSeconds() >= 2) {
                            if (!ran) {
                                timer.resetTimer();
                                FEED();
                                ran = true;
                            }
                        }
                        ledCpos = 1;
                    }
                    if ((ran && timer.getElapsedTime() >= shootWait) || !shooterOn) {
                        RESET_SHOOTER_TURRET();
                        RESET_INTAKE();
                        ran = false;
                        ran2 = false;
                        if (gate) {
                            if (!intakedMid) setPathState(2);
                            else if (!intakedGate) setPathState(4);
                            else if (!intakedClose) setPathState(1);
                            else if (!intakedFar) setPathState(3);
                            else if (matchTime.isLessThan(3)) setPathState(5);
                        } else {
                            if (!intakedClose) setPathState(1);
                            else if (!intakedMid) setPathState(2);
                            else if (!intakedFar) setPathState(3);
                            else if (matchTime.isLessThan(3)) setPathState(5);
                        }
                    }
                }
                break;
            case 1:
                if (!intakeCloseStarted) {
                    reached2 = false;
                    wheelSpeed = 0.9;
                    INTAKE();
                    follower.followPath(intakeClose, true);
                    shootCloseStarted = false;
                    intakedClose = true;
                    intakeCloseStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RC.intakeClosePose, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BC.intakeClosePose, 5, 5)) reached = true;
                if (reached && !ran2) {
                    timer.resetTimer();
                    ran2 = true;
                }
                if (ran2 && timer.getElapsedTime() >= intakeWait) {
                    wheelSpeed = 1;
                    RESET_INTAKE();
                    ran = false;
                    ran2 = false;
                    intakedClose = true;
                    setPathState(0);
                }
                break;
            case 2:
                if (!intakeMidStarted) {
                    reached2 = false;
                    INTAKE();
                    follower.followPath(intakeMid, true);
                    shootCloseStarted = false;
                    intakeMidStarted = true;
                }
                if (reached && !ran2) {
                    timer.resetTimer();
                    ran2 = true;
                }
                if (ran2 && timer.getElapsedTime() >= intakeWait) {
                    wheelSpeed = 1;
                    RESET_INTAKE();
                    ran = false;
                    ran2 = false;
                    intakedClose = true;
                    setPathState(0);
                }
                break;
            case 3:
                if (!intakeFarStarted) {
                    reached2 = false;
                    wheelSpeed = 1;
                    INTAKE();
                    follower.followPath(intakeFar, true);
                    shootCloseStarted = false;
                    intakedFar = true;
                    intakeFarStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RC.intakeFarPose, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BC.intakeFarPose, 5, 5)) reached = true;
                if (reached && !ran2) {
                    timer.resetTimer();
                    ran2 = true;
                }
                if (ran2 && timer.getElapsedTime() >= intakeWait) {
                    wheelSpeed = 1;
                    RESET_INTAKE();
                    ran = false;
                    ran2 = false;
                    intakedClose = true;
                    setPathState(0);
                }
                break;
            case 4:
                if (!intakeGateStarted) {
                    reached2 = false;
                    wheelSpeed = 1;
                    indexer.setPower(1);
                    INTAKE();
                    follower.followPath(intakeGate, true);
                    shootCloseStarted = false;
                    intakedGate = true;
                    intakeGateStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.resetTimer();
                        ran2 = true;
                    }
                    if (timer.getElapsedTime() > gateWait) {
                        wheelSpeed = 1;
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
                    RESET_INTAKE();
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;

        }
    }

    public void farStates() {
        switch (pathState) {
            case 0:
                if (!shootStarted) {
                    RESET_INTAKE();
                    shooterSS.stopBackSpin();
                    if (shooterOn) shooterSS.shooterOn(true);
                    if (turretOn) turretSS.turretOn(true);
                    ledCpos = 0.388;
                    follower.followPath(shootFar, true);
                    shot = 0;
                    timer.resetTimer();
                    shootStarted = true;
                    reached = false;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RF.startPose, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BF.startPose, 5, 5)) reached = true;
                if (reached) {
                    if (shooterR.getVelocity() >= shooterSS.getTargetVelocity()) {
                        if (timer.getElapsedTimeSeconds() >= 1.0) {
                            FEED();
                            shot++;
                            timer.resetTimer();
                        }
                    }
                    if (shot >= 3 || !shooterOn || timer.getElapsedTimeSeconds() > 9) {
                        RESET_SHOOTER_TURRET();
                        shot = 0;
                        shootStarted = false;
                        if (humanPlayer && !humanStarted) setPathState(1);
                        else if (!leaveStarted) setPathState(2);
                    }
                }
                break;
            case 1:
                if (!humanStarted) {
                    shootStarted = false;
                    shot = 0;
                    wheelSpeed = 0.6;
                    RESET_INTAKE();
                    INTAKE();
                    follower.followPath(human, true);
                    timer2.resetTimer();
                    reached = false;
                    humanStarted = true;
                }
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RF.humanPose, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BF.humanPose, 5, 5)) reached = true;
                if (reached && timer2.getElapsedTimeSeconds() > humanWait) {
                    wheelSpeed = 0.85;
                    RESET_INTAKE();
                    setPathState(0);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(leave, true);
                    setPathState(-1);
                }
                break;
        }
    }


    public void INTAKE() {
        pivotCpos = 0.55;// no
        intake.setPower(1);
        if (!beams.getState()) indexer.setPower(0);
        else if (indexerOn) indexer.setPower(1);
        shooterSS.backSpin(-600);
    }

    public void OUTTAKE() {
        indexerOn = true;// no
        pivotCpos = 0.55;
        indexer.setPower(1);
        intake.setPower(-1);// :P
    }
    public void RESET_INTAKE() {
        pivotCpos = 0.45;
        indexer.setPower(0);
        intake.setPower(0);
        shooterVelo = 0;
        indexerOn = true;
    }
    public void FEED() {
        indexerOn = true;
        pivotCpos = 0.45;
        indexer.setPower(1);
        intake.setPower(1);
    }
    public void RESET_SHOOTER_TURRET() {
        shooterSS.shooterOn(false);
        turretSS.turretOn(false); // i could so go for a hot coca right now
        ledCpos = 0.667;
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .prompt("human", () -> {
                    if (prompter.get("start_pos") == MainV1E.StartPos.FAR) return new BooleanPrompt("Intake human player", true);
                    return null; // Skip
                })
                .prompt("gate", () -> {
                    if (prompter.get("start_pos") == MainV1E.StartPos.CLOSE) return new BooleanPrompt("Use the gate", false);
                    return null; // Skip
                })
                .onComplete(this::onPromptsComplete);
        MainV1E.lastAutoPos = null;
        timer = new Timer();
        timer2 = new Timer();
        shotTimer = new Timer();
        loopTime = new ElapsedTime();
        loopTime.reset();
        // hardware
        turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        // hi
        beams = hardwareMap.get(DigitalChannel.class, "bb"); // bby
        // gamepads
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
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
        // directions
        beams.setMode(DigitalChannel.Mode.INPUT);
        indexer.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // limits
        hood.scaleRange(0, 0.38);
        pivot.scaleRange(0, 0.4);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.667); // david is mean he is mad
        strips.setPosition(stripsCpos = initGameStrips); // white
        pinpoint.recalibrateIMU();
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // subsystems
        shooterSS = new ShooterSS(shooterPID, shooterR, shooterL, hoodR, hoodL);
        shooterSS.setPoses(MainV2.getShooterLUT(), 15.1, 124.9, MainV2.getHoodLut(), 15.1, 124.9);
        shooterSS.update(follower);
    }

    public void onPromptsComplete() {
        alliance = prompter.get("alliance");
        startPos = prompter.get("start_pos");
        gate = prompter.getOrDefault("gate", false);
        humanPlayer = prompter.getOrDefault("human", false);
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(RF.startPose);
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(BF.startPose);
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(RC.startPose);
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(BC.startPose);
        }
        redSide = alliance == MainV1E.Alliance.RED;
        MainV2.redSide = redSide;
        MainV1E.redSideS = redSide;
        turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, indexer, PIDTuneTurret.TPR, PIDTuneTurret.ratio, redSide ? turretOffsetR : turretOffsetB, MainV1E.lastTurretPos);
        turretSS.setRedSide(redSide);
        shooterSS.setRedSide(redSide);
        turretSS.setWrapAngles(-180, 180);
        turretSS.update(follower);
        buildPaths();
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.addData(true, "Alliance", alliance);
        telemetryM.addData(true, "Starting pos", startPos);
        telemetryM.addData(true, "Using gate", gate);
        telemetryM.addData(true, "Starting pos", humanPlayer);
        telemetryM.update();
    }
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
    }

    @Override
    public void loop() {
        Pose bluePos = new Pose(11, 137, Math.toRadians(0));
        Pose redPos = new Pose(133, 137, Math.toRadians(0));
        turretPID.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        turretSS.updatePID(turretPID, PIDTuneTurret.F);
        turretSS.setTurretOffset(alliance == MainV1E.Alliance.RED ? turretOffsetR : turretOffsetB);
        shooterSS.updatePID(shooterPID); // woahhh
        shooterSS.setShooterOffset(shooterOffset);
        shooterSS.setPoses(bluePos, redPos); // woah
        turretSS.setPoses(bluePos, redPos);
        if(!turretOn) turretSS.turretOn(false);
        if(!shooterOn) shooterSS.shooterOn(false);
        follower.setMaxPower(wheelSpeed);
        if (startPos == MainV1E.StartPos.FAR) farStates();
        if (startPos == MainV1E.StartPos.CLOSE) closeStates();
        // servos
        pivot.setPosition(pivotCpos);
        led.setPosition(ledCpos);
        // shooter code
        shooterSS.alignShooter();
        shooterSS.update(follower);
        // turret code
        if (turretSS.alignTurret() == 0 && turretTpos != 0) turretSS.updateTurretTpos(turretTpos);
        else if (turretSS.alignTurret() != 0) turretTpos = turretSS.turretTpos;
        turretSS.alignTurret();
        turretSS.update(follower);
        follower.update();
        MainV1E.lastAutoPos = follower.getPose();
        MainV1E.lastTurretPos = turretSS.getCurrentPos();
        // telemetry
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addData(true, "loop times", loopTime.milliseconds());
        telemetryM.addData(true, "timer", timer.getElapsedTime());
        telemetryM.addData(true, "pivot", pivot.getPosition());
        telemetryM.addData(true, "hood", hood.getPosition());
        telemetryM.addData(true, "indexer", indexer.getPower());
        telemetryM.addData(true, "led", led.getPosition());
        telemetryM.addData(true, "strips", strips.getPosition());
        telemetryM.addData(true, "gameTimer", gameTimer.getElapsedTimeSeconds());
        telemetryM.addLine(true, turretSS.telemetry());
        telemetryM.addLine(true, shooterSS.telemetry());
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
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        // MainV1E.lastAutoPos = follower.getPose(); // COMMENT THIS IF TELEOP AFTER AUTO NO WORKY
    }
}
