package org.firstinspires.ftc.teamcode.auto.offseason;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;// david is big for eati g so ,ich pizza
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.offseason.points.Points;
import org.firstinspires.ftc.teamcode.auto.v2_2.points.BC;
import org.firstinspires.ftc.teamcode.auto.v2_2.points.BF;
import org.firstinspires.ftc.teamcode.auto.v2_2.points.RC;
import org.firstinspires.ftc.teamcode.auto.v2_2.points.RF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV2;
import org.firstinspires.ftc.teamcode.teleOp.MainV3;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDDualTuneTurret;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

// @Disabled
@Config
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
    CombinedServo turret; // 2x axon minimoni
    // pids
    PIDController turretPID;
    PIDFCoefficients shooterPID;
    // subsystems
    TurretSS turretSS;
    ShooterSS shooterSS;
    // positions
    public static double pivotCpos = 0.1;
    public static double hoodCpos = 0;
    public static double ledCpos = 0.667;
    public static double stopperCpos = 0.5;
    public static double stripsCpos = 0.611;
    public static double turretTpos = 0;
    public static double shooterVelo = 0; // update servos r kissing
    public static double initGameStrips = 0.75;
    // misc
    private double wheelSpeed = 1;
    public static boolean turretOn = true;
    boolean indexerOn = true;
    public static double turretOffsetR = 5; // kabam
    public static double turretOffsetB = -2; // kabamkavhow
    public static double shooterOffset = -18;
    public static boolean debugMode = true;
    public static boolean redSide = false;
    public static int intakeWait = 400;
    public static int humanWait = 2000;
    public static int gateWait = 500;
    public static int shootWait = 1300;
    DigitalChannel beams;
    ColorRangeSensor c1;
    ColorRangeSensor c2;
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
    private PathChain scorePre, intakeClose, scoreClose, intakeMid, gateOpen, scoreMid, intakeFar, scoreFar, park;
    // close
    boolean shootS = false;
    boolean intakeCloseS = false;
    // shoot close
    boolean intakeMidS = false;
    boolean gateOpenS = false;
    // shoot mid
    boolean intakeFarS = false;
    // shoot close far
    boolean parkS = false;
    boolean intakeGateS = false;
    boolean intakedClose = false;
    boolean intakedMid = false;
    boolean intakedFar = false;
    boolean intakedGate = false;
    // far
    private PathChain shootPre, intakeeFar, shootFar, intakeeMid, shootMid, intakeeClose, shootClose, parkk, leave;
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
                        Points.BF.start,
                        Points.BF.leave
                ))
                .setConstantHeadingInterpolation(Points.BF.leave.getHeading())
                .build();
        shootPre = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.start,
                        Points.BF.shootPre
                ))
                .setConstantHeadingInterpolation(Points.BF.shootPre.getHeading())
                .build();
        intakeeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.BF.shootPre,
                        Points.BF.intakeFarControl,
                        Points.BF.intakeFar
                ))
                .setConstantHeadingInterpolation(Points.BF.intakeFar.getHeading())
                .build();
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.intakeFar,
                        Points.BF.shootFar
                ))
                .setConstantHeadingInterpolation(Points.BF.shootFar.getHeading())
                .build();
        intakeeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.BF.shootFar,
                        Points.BF.intakeMidControl,
                        Points.BF.intakeMid
                ))
                .setConstantHeadingInterpolation(Points.BF.intakeMid.getHeading())
                .build();
        shootMid = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.intakeMid,
                        Points.BF.shootMid
                ))
                .setConstantHeadingInterpolation(Points.BF.shootMid.getHeading())
                .build();
        intakeeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.shootMid,
                        Points.BF.intakeClose
                ))
                .setConstantHeadingInterpolation(Points.BF.intakeClose.getHeading())
                .build();
        shootClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.intakeClose,
                        Points.BF.shootClose
                ))
                .setConstantHeadingInterpolation(Points.BF.shootClose.getHeading())
                .build();
        parkk = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BF.shootClose,
                        Points.BF.park
                ))
                .setConstantHeadingInterpolation(Points.BF.park.getHeading())
                .build();
    }
    private void buildRedFar() {
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.start,
                        Points.RF.leave
                ))
                .setConstantHeadingInterpolation(Points.RF.leave.getHeading())
                .build();
        shootPre = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.start,
                        Points.RF.shootPre
                ))
                .setConstantHeadingInterpolation(Points.RF.shootPre.getHeading())
                .build();
        intakeeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.RF.shootPre,
                        Points.RF.intakeFarControl,
                        Points.RF.intakeFar
                ))
                .setConstantHeadingInterpolation(Points.RF.intakeFar.getHeading())
                .build();
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.intakeFar,
                        Points.RF.shootFar
                ))
                .setConstantHeadingInterpolation(Points.RF.shootFar.getHeading())
                .build();
        intakeeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.RF.shootFar,
                        Points.RF.intakeMidControl,
                        Points.RF.intakeMid
                ))
                .setConstantHeadingInterpolation(Points.RF.intakeMid.getHeading())
                .build();
        shootMid = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.intakeMid,
                        Points.RF.shootMid
                ))
                .setConstantHeadingInterpolation(Points.RF.shootMid.getHeading())
                .build();
        intakeeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.shootMid,
                        Points.RF.intakeClose
                ))
                .setConstantHeadingInterpolation(Points.RF.intakeClose.getHeading())
                .build();
        shootClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.intakeClose,
                        Points.RF.shootClose
                ))
                .setConstantHeadingInterpolation(Points.RF.shootClose.getHeading())
                .build();
        parkk = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RF.shootClose,
                        Points.RF.park
                ))
                .setConstantHeadingInterpolation(Points.RF.park.getHeading())
                .build();
    }
    private void buildBlueClose() {
        scorePre = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.start,
                        Points.BC.shootPre
                ))
                .setConstantHeadingInterpolation(Points.BC.shootPre.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.shootPre,
                        Points.BC.intakeClose
                ))
                .setConstantHeadingInterpolation(Points.BC.intakeClose.getHeading())
                .build();
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.intakeClose,
                        Points.BC.shootClose
                ))
                .setLinearHeadingInterpolation(Points.BC.intakeClose.getHeading(), Points.BC.shootClose.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.BC.shootClose,
                        Points.BC.intakeMidControl,
                        Points.BC.intakeMid
                ))
                .setConstantHeadingInterpolation(Points.BC.intakeMid.getHeading())
                .build();
        gateOpen = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.BC.intakeMid,
                        Points.BC.gateControl,
                        Points.BC.gate
                ))
                .setConstantHeadingInterpolation(Points.BC.gate.getHeading())
                .build();
        scoreMid = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.gate,
                        Points.BC.shootMid
                ))
                .setLinearHeadingInterpolation(Points.BC.gate.getHeading(), Points.BC.shootMid.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.BC.shootMid,
                        Points.BC.intakeFarControl,
                        Points.BC.intakeFar
                ))
                .setConstantHeadingInterpolation(Points.BC.intakeFar.getHeading())
                .build();
        scoreFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.intakeFar,
                        Points.BC.shootFar
                ))
                .setLinearHeadingInterpolation(Points.BC.intakeFar.getHeading(), Points.BC.shootFar.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.BC.shootFar,
                        Points.BC.park
                ))
                .setConstantHeadingInterpolation(Points.BC.park.getHeading())
                .build();
    }
    private void buildRedClose() {
        scorePre = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.start,
                        Points.RC.shootPre
                ))
                .setConstantHeadingInterpolation(Points.RC.shootPre.getHeading())
                .build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.shootPre,
                        Points.RC.intakeClose
                ))
                .setConstantHeadingInterpolation(Points.RC.intakeClose.getHeading())
                .build();
        scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.intakeClose,
                        Points.RC.shootClose
                ))
                .setConstantHeadingInterpolation(Points.RC.shootClose.getHeading())
                .build();
        intakeMid = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.RC.shootClose,
                        Points.RC.intakeMidControl,
                        Points.RC.intakeMid
                ))
                .setConstantHeadingInterpolation(Points.RC.intakeMid.getHeading())
                .build();
        gateOpen = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.RC.intakeMid,
                        Points.RC.gateControl,
                        Points.RC.gate
                ))
                .setConstantHeadingInterpolation(Points.RC.gate.getHeading())
                .build();
        shootMid = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.gate,
                        Points.RC.shootMid
                ))
                .setConstantHeadingInterpolation(Points.RC.shootMid.getHeading())
                .build();
        intakeFar = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Points.RC.shootMid,
                        Points.RC.intakeFarControl,
                        Points.RC.intakeFar
                ))
                .setConstantHeadingInterpolation(Points.RC.intakeFar.getHeading())
                .build();
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.intakeFar,
                        Points.RC.shootFar
                ))
                .setConstantHeadingInterpolation(Points.RC.shootFar.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        Points.RC.shootFar,
                        Points.RC.park
                ))
                .setConstantHeadingInterpolation(Points.RC.park.getHeading())
                .build();
    }


    public void closeStates() {
        switch (pathState) {
            case 0:
                if (!shootS) {
                    shootPathInit(scorePre);
                    shootS = true;
                }
                shootPathLogic(1, Points.BC.shootPre, Points.RC.shootPre);
                break;
            case 1:
                if (!intakeCloseS) {
                    intakePathInit(intakeClose);
                    intakeCloseS = true;
                }
                intakePathLogic(2, Points.BC.intakeClose, Points.RC.intakeClose);
                break;
            case 2:
                if (!shootS) {
                    shootPathInit(scoreClose);
                    shootS = true;
                }
                shootPathLogic(3, Points.BC.shootClose, Points.RC.shootClose);
                break;
            case 3:
                if (!intakeMidS) {
                    intakePathInit(intakeMid);
                    intakedMid = false;
                    intakeMidS = true;
                }
                intakePathLogic(4, Points.BC.intakeMid, Points.RC.intakeMid);
                break;
            case 4:
                if(!gateOpenS) {
                    follower.followPath(gateOpen, true);
                    ran2 = false;
                    gateOpenS = true;
                }
                if (!follower.isBusy() && gateOpenS) {
                    if (!ran2) {
                        timer.resetTimer();
                        ran2 = true;
                    }
                    if(shooterOn) shooterSS.align();
                    RESET_INTAKE();
                    if(timer.getElapsedTime() >= gateWait && ran2) setPathState(5);
                }
                break;
            case 5:
                if (!shootS) {
                    shootPathInit(scoreMid);
                    shootS = true;
                }
                shootPathLogic(6, Points.BC.shootMid, Points.RC.shootMid);
                break;
            case 6:
                if (!intakeFarS) {
                    intakePathInit(intakeFar);
                    intakedFar = false;
                    intakeFarS = true;
                }
                intakePathLogic(7, Points.BC.intakeFar, Points.RC.intakeFar);
                break;
            case 7:
                if (!shootS) {
                    shootPathInit(scoreFar);
                    shootS = true;
                }
                shootPathLogic(8, Points.BC.shootFar, Points.RC.shootFar);
                break;
            case 8:
                if(!parkS) {
                    follower.followPath(park, true);
                    parkS = true;
                }
                if (!follower.isBusy() && parkS) {
                    RESET_SHOOTER_TURRET();
                    RESET_INTAKE();
                    setPathState(-1);
                }
                break;
        }
    }

    public void farStates() {
        switch (pathState) {
            /*
            case 0:
                if (!shootStarted) {
                    RESET_INTAKE();
                    if (turretOn) turretSS.align();
                    follower.followPath(shootFar, true);
                    shot = 0;
                    timer.resetTimer();
                    shootStarted = true;
                    reached = false;
                }
                if (shooterOn) shooterSS.align();
                if (alliance == MainV1E.Alliance.RED && follower.atPose(RF.startPose, 5, 5)) reached = true;
                if (alliance == MainV1E.Alliance.BLUE && follower.atPose(BF.startPose, 5, 5)) reached = true;
                if (reached) {
                    if (shooterSS.atTarget()) {
                        if (timer.getElapsedTimeSeconds() >= 1.0) {
                            FEED();
                            shot++;
                            timer.resetTimer();
                        } else RESET_INTAKE();
                    }
                    if (shot >= 3 || !shooterOn || timer.getElapsedTimeSeconds() > 9) {
                        RESET_SHOOTER_TURRET();
                        shot = 0;
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

             */
        }
    }


    public void INTAKE() {
        pivotCpos = 0.15;
        stopperCpos = 0.5;
        if (indexerOn) indexer.setPower(0.9);
        intake.setPower(1);
        if (!beams.getState()) {
            indexerOn = false;
            indexer.setPower(0);
        }
        if (!beams.getState() && c2.getDistance(DistanceUnit.CM) < 10 && c1.getDistance(DistanceUnit.CM) < 10)  {
            shooterSS.setLeds(0.667);
        }
    }

    public void OUTTAKE() {
        indexerOn = true;
        pivotCpos = 0.15;
        stopperCpos = 0.5;
        indexer.setPower(-1);
        intake.setPower(-1);
    }
    public void RESET_INTAKE() {
        pivotCpos = 0.1;
        stopperCpos = 0.5;
        indexer.setPower(0);
        intake.setPower(0);
        shooterSS.reset();
        indexerOn = true;
    }
    public void FEED() {
        indexerOn = true;
        pivotCpos = 0.1;
        stopperCpos = 0;
        indexer.setPower(0.9);
        intake.setPower(1);
    }
    public void RESET_SHOOTER_TURRET() {
        shooterSS.reset();
        turretSS.reset();
    }
    // path logic
    public void shootPathInit(PathChain path) {
        follower.followPath(path, true);
    }
    public void shootPathLogic(int pathState, Pose shootBC, Pose shootRC) {
        if (shooterOn) shooterSS.align();
        if (alliance == MainV1E.Alliance.RED && follower.atPose(shootRC, 4, 4)) reached2 = true;
        if (alliance == MainV1E.Alliance.BLUE && follower.atPose(shootBC, 4, 4)) reached2 = true;
        if (reached2 && shootS) {
            if (turretOn) turretSS.align();
            if (shooterSS.atTarget()) {
                if (!ran2) {
                    timer2.resetTimer();
                    ran2 = true;
                }
                if (timer2.getElapsedTime() >= 450) {
                    if (!ran) {
                        timer.resetTimer();  // check this line its sus
                        FEED();
                        ran = true;
                    }
                }
            }
            if ((ran && timer.getElapsedTime() >= shootWait) || !shooterOn) {
                RESET_SHOOTER_TURRET();
                RESET_INTAKE();
                ran = false;
                ran2 = false;
                setPathState(pathState);
            }
        }
    }

    public void intakePathInit(PathChain path) {
        reached2 = false;
        reached = false;
        wheelSpeed = 0.9;
        ran2 = false;
        INTAKE();
        follower.followPath(path, true);
        shootS = false;
    }
    public void intakePathLogic(int pathState, Pose intakeBC, Pose intakeRC) {
        if (alliance == MainV1E.Alliance.RED && follower.atPose(intakeRC, 5, 5)) reached = true;
        if (alliance == MainV1E.Alliance.BLUE && follower.atPose(intakeBC, 5, 5)) reached = true;
        if (reached && !ran2) {
            timer.resetTimer();
            ran2 = true;
        }
        if (ran2 && timer.getElapsedTime() >= intakeWait) {
            wheelSpeed = 1;
            RESET_INTAKE();
            ran = false;
            ran2 = false;
            setPathState(pathState);
        } else INTAKE();
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
        loopTime = new ElapsedTime();
        loopTime.reset();
        // hardware
        turretPID = new PIDController(Math.sqrt(PIDDualTuneTurret.FAR.P), PIDDualTuneTurret.FAR.I, PIDDualTuneTurret.FAR.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
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
        CachingServo turret1 = new CachingServo(hardwareMap.get(Servo.class, "turret1")); // 1x axon mini
        CachingServo turret2 = new CachingServo(hardwareMap.get(Servo.class, "turret2")); // 1x axon mini
        turret = new CombinedServo(turret1, turret2); // 2x axon minis
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        strips = new CachingServo(hardwareMap.get(Servo.class, "strips")); // 4x gobilda strip RGB lights
        stopper = new CachingServo(hardwareMap.get(Servo.class, "stopper")); // 1x axon mini
        // sensors
        beams = hardwareMap.get(DigitalChannel.class, "bb"); // bby
        c1 = hardwareMap.get(ColorRangeSensor.class, "c1");
        c2 = hardwareMap.get(ColorRangeSensor.class, "c2");
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
        stopper.scaleRange(0.42, 1);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.1);
        led.setPosition(ledCpos = 0.667); // david is mean he is mad
        strips.setPosition(stripsCpos = initGameStrips); // white
        stopper.setPosition(stopperCpos = 0.5);
        pinpoint.recalibrateIMU();
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // subsystems
        turretSS = new TurretSS(turret, indexer, PIDTuneTurret.pidf, MainV1E.lastTurretPos);
        shooterSS = new ShooterSS(new CombinedDcMotorEx(shooterR, shooterL), hood, led, shooterPID);
        shooterSS.setPoses(MainV3.getShooterLUT(), 26.1, 64.9, MainV3.getHoodLut(), 26.1, 64.9);
        shooterSS.update(follower);
        turretSS.update(follower);
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
        turretSS.setOffset(redSide ? turretOffsetR : turretOffsetB);
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
        Pose bluePos = new Pose(9, 138, 135); // BYE
        Pose redPos = new Pose(138, 138, 45);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P, PIDTuneShooterSdk.I, PIDTuneShooterSdk.D, PIDTuneShooterSdk.F);
        turretSS.setOffset(redSide ? turretOffsetR : turretOffsetB);
        shooterSS.updatePID(shooterPID);
        shooterSS.setOffset(shooterOffset);
        shooterSS.setPose(bluePos, redPos);
        turretSS.setPoses(bluePos, redPos);
        follower.setMaxPower(wheelSpeed);
        if (startPos == MainV1E.StartPos.FAR) farStates();
        if (startPos == MainV1E.StartPos.CLOSE) closeStates();
        // servos
        pivot.setPosition(pivotCpos);
        led.setPosition(ledCpos);
        stopper.setPosition(stopperCpos);
        // shooter code
        shooterSS.update(follower);
        // turret code
        if (turretSS.getTarget() == 0 && turretTpos != 0) turretSS.setTarget(turretTpos);
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
