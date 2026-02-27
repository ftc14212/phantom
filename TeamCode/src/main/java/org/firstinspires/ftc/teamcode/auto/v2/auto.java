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
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.v2.points.BC;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV1;
import org.firstinspires.ftc.teamcode.teleOp.MainV2;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooter;
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
    public static double indexerCpos = 0;
    public static double ascendCpos = 0;
    public static double ledCpos = 0.611;
    public static double stripsCpos = 0.611;
    public static double turretTpos = 0;
    public static double shooterVelo = 0; // update servos r kissing
    public static double midGameStrips = 0.7;
    public static double initGameStrips = 0.75;
    // misc
    private double wheelSpeed = 1;
    public static boolean turretOn = true;
    boolean indexerOn = true;
    public static double turretOffset = 3; // kabam
    public static double shooterOffset = -18;
    public static boolean debugMode = true;
    public static boolean redSide = false;
    public static int intakeWait = 500;
    public static int gateWait = 2000;
    public static int shootWait = 1300;
    DigitalChannel beams;
    private final Prompter prompter = new Prompter(this);
    private MainV1E.Alliance alliance = MainV1E.Alliance.RED;
    private MainV1E.StartPos startPos = MainV1E.StartPos.FAR;
    TimerEx matchTime = new TimerEx(30); // 30 second autonomous
    private int pathState;
    // close
    private PathChain scoreClose, intakeClose, intakeMid, intakeFar, intakeGate, park;
    // blue close
    boolean shootCloseStarted = false;
    boolean intakeCloseStarted = false;
    boolean intakeMidStarted = false;
    boolean intakeFarStarted = false;
    boolean intakeGateStarted = false;
    boolean intakedClose = false;
    boolean intakedMid = false;
    boolean intakedFar = false;
    boolean intakedGate = false;
    public void buildPaths() {
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.BLUE) buildBlueClose();
            // if (alliance == MainV1E.Alliance.RED) buildRedClose();
        }
        if (startPos == MainV1E.StartPos.FAR) {
            // if (alliance == MainV1E.Alliance.BLUE) buildBlueFar();
            // if (alliance == MainV1E.Alliance.RED) buildRedFar();
        }
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
                        follower.getPose(),
                        BC.intakeMidControlPose,
                        BC.intakeMidPose
                ))
                .setConstantHeadingInterpolation(BC.intakeMidPose.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
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
                        follower.getPose(),
                        RC.intakeMidControlPose,
                        RC.intakeMidPose
                ))
                .setConstantHeadingInterpolation(RC.intakeMidPose.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
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
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .prompt("gate", new BooleanPrompt("Enable Auto Score?", gate))
                .onComplete(this::onPromptsComplete);
        MainV1E.lastAutoPos = null;
        timer = new Timer();
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
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // reset pos
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // limits
        hood.scaleRange(0, 0.38);
        pivot.scaleRange(0, 0.4);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611); // david is mean he is mad
        strips.setPosition(stripsCpos = initGameStrips); // white
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // subsystems
        turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, indexer, PIDTuneTurret.TPR, PIDTuneTurret.ratio, turretOffset, MainV1E.lastTurretPos);
        shooterSS = new ShooterSS(shooterPID, shooterR, shooterL, hoodR, hoodL);
        shooterSS.setPoses(MainV2.getShooterLUT(), 15.1, 124.9, MainV2.getHoodLut(), 15.1, 124.9);
        turretSS.setWrapAngles(-180, 180);
    }
    public void onPromptsComplete() {
        alliance = prompter.get("alliance");
        startPos = prompter.get("start_pos");
        gate = prompter.get("gate");
        if (startPos == MainV1E.StartPos.FAR) {
            // if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(startPoseRF);
            // if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(startPoseBF);
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(RC.startPose);
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(BC.startPose);
        }
        redSide = alliance == MainV1E.Alliance.RED;
        intakedGate = redSide;
        MainV1.redSide = alliance == MainV1E.Alliance.RED;
        buildPaths();
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.addData(true, "Alliance", alliance);
        telemetryM.addData(true, "Starting pos", startPos);
        telemetryM.update();
    }

    @Override
    public void loop() {
        Pose bluePos = new Pose(11, 137, Math.toRadians(0));
        Pose redPos = new Pose(133, 137, Math.toRadians(0));
        turretPID.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        turretSS.updatePID(turretPID, PIDTuneTurret.F);
        turretSS.setTurretOffset(turretOffset);
        shooterSS.updatePID(shooterPID); // woahhh
        shooterSS.setShooterOffset(shooterOffset);
        shooterSS.setPoses(bluePos, redPos); // woah
        turretSS.setPoses(bluePos, redPos);
        if (!beams.getState() && !indexerOn) indexerCpos = 0;
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        follower.setMaxPower(wheelSpeed);
        if (startPos == MainV1E.StartPos.FAR) farStates();
        if (startPos == MainV1E.StartPos.CLOSE) closeStates();
        // servos
        pivot.setPosition(pivotCpos);
        hood.setPosition(hoodCpos);
        indexer.setPower(indexerCpos);
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
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        MainV1E.lastAutoPos = follower.getPose(); // COMMENT THIS IF TELEOP AFTER AUTO NO WORKY
        MainV1E.lastTurretPos = turretSS.getCurrentPos();
    }
}
