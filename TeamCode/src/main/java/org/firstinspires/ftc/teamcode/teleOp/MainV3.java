/***
 * MAIN V3
 * @author David Grieas - 14212 MetroBotics
 * coding for our V2.2 Phantom - june 20th
***/
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import java.util.List;

import org.firstinspires.ftc.teamcode.utils.CachingCRServo;
import org.firstinspires.ftc.teamcode.utils.CachingDcMotorEx;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@Configurable
@TeleOp(name="Main v3", group=".ftc14212")
public class MainV3 extends OpMode {
    /**
     * MAIN V3 BY DAVID
     * @author David Grieas, Iza Sikorski - 14212 MetroBotics
    **/
    // positions
    public static double pivotCpos = 0;
    public static double stopperCpos = 1;
    public static double stripsCpos = 0.611;
    public static double turretTpos = 0;
    public static double endGameStrips = 0.89;
    public static double midGameStrips = 0.7;
    public static double initGameStrips = 0.75;
    // misc
    public static double wheelSpeed = 1;
    public static boolean turretOn = true;
    public static boolean shooterOn = true;
    boolean indexerOn = true;
    public static double idle = 900;
    // timers
    ElapsedTime loopTime;
    // odometry
    public static boolean odoDrive = true;
    // config stuff
    public static boolean redSide;
    public static boolean debugMode = false;
    public static double turretOffsetR = 0;
    public static double turretOffsetB = 0;
    public static double shooterOffset = -16;
    private final Prompter prompter = new Prompter(this);
    // hardware
    private List<LynxModule> allHubs;
    private TelemetryM telemetryM;
    private Follower follower;
    DigitalChannel beams;
    ColorRangeSensor c1;
    ColorRangeSensor c2;
    // gamepads
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    // motors
    CachingDcMotorEx leftFront; // 312 rpm --> 468 rpm
    CachingDcMotorEx leftRear; // 312 rpm --> 468 rpm
    CachingDcMotorEx rightFront; // 312 rpm --> 468 rpm
    CachingDcMotorEx rightRear; // 312 rpm --> 468 rpm
    CachingDcMotorEx shooterL; // 6000 rpm --> 4800 rpm
    CachingDcMotorEx shooterR; // 6000 rpm --> 4800 rpm
    CachingDcMotorEx intake; // 1150 rpm --> 638.9 - 805.6 rpm
    CachingDcMotorEx indexer; // 1620 rpm --> 900 rpm
    // servos
    CachingServo pivot; // 1x axon max
    CachingServo stopper; // 1x axon mini
    CombinedServo hood; // 2x axon mini
    CachingServo led; // 2x gobilda led lights RGB
    CachingServo strips; // 4x gobilda strip RGB lights
    CombinedCRServo turret; // 2x axon mini
    // random
    MainV1E.Alliance alliance;
    MainV1E.StartPos startPos;
    Timer gameTimer = new Timer();
    // pids
    PIDFCoefficients shooterPID;
    // subsystems
    TurretSS turretSS;
    ShooterSS shooterSS;
    @Override
    public void init() {
        if (MainV1E.lastAutoPos == null) prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .onComplete(this::onPromptsComplete);
        // hardware
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        LynxUtils.initLynx(hardwareMap);
        // gamepads
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        // motors
        leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront")); // 312 rpm --> 468 rpm
        leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear")); // 312 rpm --> 468 rpm
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront")); // 312 rpm --> 468 rpm
        rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear")); // 312 rpm --> 468 rpm
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
        // sensors
        beams = hardwareMap.get(DigitalChannel.class, "bb");
        c1 = hardwareMap.get(ColorRangeSensor.class, "c1");
        c2 = hardwareMap.get(ColorRangeSensor.class, "c2");
        // Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        // limelight3A.setPollRateHz(50);
        // limits
        hood.scaleRange(0, 0.37);
        pivot.scaleRange(0, 0.375);
        stopper.scaleRange(0.42, 1);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        if (MainV1E.lastAutoPos != null) follower.setStartingPose(new Pose(MainV1E.lastAutoPos.getX(), MainV1E.lastAutoPos.getY(), MainV1E.lastAutoPos.getHeading()));
        MainV1E.lastAutoPos = null;
        hood.setPosition(0);
        pivot.setPosition(pivotCpos = 0);
        led.setPosition(0.611);
        stopper.setPosition(stopperCpos = 0.5);
        strips.setPosition(stripsCpos = initGameStrips);
        pinpoint.recalibrateIMU();
        // subsystems
        turretSS = new TurretSS(turret, indexer, PIDTuneTurret.pidf, MainV1E.lastTurretPos);
        shooterSS = new ShooterSS(new CombinedDcMotorEx(shooterR, shooterL), hood, led, shooterPID);
        shooterSS.setPoses(getShooterLUT(), 26.1, 64.9, getHoodLut(), 26.1, 64.9);
        turretSS.setWrapAngles(-170, 170);
        shooterSS.setIdle(idle);
        turretSS.update(follower);
        shooterSS.update(follower);
        // misc
        loopTime = new ElapsedTime();
        follower.update();
        beams.setMode(DigitalChannel.Mode.INPUT);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MainV1E.lastTurretPos = -999;
        if (MainV1E.redSideS) {
            redSide = true;
            turretSS.setRedSide(true);
            shooterSS.setRedSide(true);
            MainV1E.redSideS = false;
        }
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        loopTime.reset();
    }

    public void onPromptsComplete() {
        alliance = prompter.get("alliance");
        startPos = prompter.get("start_pos");
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(89.4, 7.8, Math.toRadians(90)));
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(new Pose(54.6, 7.8, Math.toRadians(90)));
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(126, 119, Math.PI - Math.toRadians(144)));
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(new Pose(18, 119, Math.toRadians(144)));
        }
        redSide = alliance == MainV1E.Alliance.RED;
        turretSS.setRedSide(redSide);
        shooterSS.setRedSide(redSide);
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.addData(true, "Alliance", alliance);
        telemetryM.addData(true, "Starting pos", startPos);
        telemetryM.update();
    }

    @Override
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void start() {
        shooterSS.reset();
        turretSS.reset();
        shooterSS.reset();
        gameTimer.resetTimer();
        follower.startTeleopDrive();
        shooterSS.skipAboveCheck();
    }
    @Override
    public void loop() {
        // poses
        Pose bluePos = new Pose(6, 138); // BYE
        Pose redPos = new Pose(138, 138);
        // debugs
        telemetryM.setDebug(debugMode);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P, PIDTuneShooterSdk.I, PIDTuneShooterSdk.D, PIDTuneShooterSdk.F);
        turretSS.setOffset(redSide ? turretOffsetR : turretOffsetB);
        shooterSS.updatePID(shooterPID);
        turretSS.updatePID(PIDTuneTurret.pidf);
        shooterSS.setOffset(shooterOffset);
        shooterSS.setPose(redPos, bluePos);
        turretSS.setPoses(bluePos, redPos);
        if (shooterOn) shooterSS.setIdle(0);
        else shooterSS.setIdle(idle);
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        // vars
        if (gameTimer.getElapsedTimeSeconds() < 100) stripsCpos = midGameStrips;
        if (gameTimer.getElapsedTimeSeconds() > 100) stripsCpos = endGameStrips;
        // status
        boolean INTAKE = gamepad1.left_trigger > 0.1;
        boolean OUTTAKE = gamepad1.right_trigger > 0.1;
        boolean FEED = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean ALIGN_SHOOT = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean RESET_SHOOTER_TURRET = (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) || (!currentGamepad2.left_bumper && previousGamepad2.left_bumper);
        boolean RESET_INTAKE = (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) || gamepad1.dpadRightWasReleased() || (!(currentGamepad1.right_trigger > 0.1) && previousGamepad1.right_trigger > 0.1) || (!(currentGamepad1.left_trigger > 0.1) && previousGamepad1.left_trigger > 0.1);
        // gamepad stuff
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        // servos
        pivot.setPosition(pivotCpos);
        strips.setPosition(stripsCpos);
        stopper.setPosition(stopperCpos);
        if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b)) turretOn = !turretOn;
        // field side
        if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
        // toggle debug
        if ((currentGamepad1.options && !previousGamepad1.options) || (currentGamepad2.options && !previousGamepad2.options)) debugMode = !debugMode;
        // movements
        if (!odoDrive) {
            // drive
            double forward = -gamepad1.left_stick_y; // forward
            double strafe = gamepad1.left_stick_x; // strafe
            double turn = gamepad1.right_stick_x;  // rotation
            // formula
            double leftFrontPower = (forward + strafe + turn) * wheelSpeed;
            double leftBackPower = (forward - strafe + turn) * wheelSpeed;
            double rightFrontPower = (forward - strafe - turn) * wheelSpeed;
            double rightBackPower = (forward + strafe - turn) * wheelSpeed;
            // power
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightBackPower);
        } else {
            follower.setMaxPower(wheelSpeed);
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        // controls
        if (INTAKE) {
            pivotCpos = 0.06;
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
        if (OUTTAKE) {
            indexerOn = true;
            pivotCpos = 0.06;
            stopperCpos = 0.5;
            indexer.setPower(-1);
            intake.setPower(-1);
        }
        if (FEED) {
            indexerOn = true;
            pivotCpos = 0;
            stopperCpos = 0;
            indexer.setPower(0.9);
            intake.setPower(1);
        } else if ((previousGamepad1.right_bumper && !currentGamepad1.right_bumper) || (previousGamepad2.right_bumper && !currentGamepad2.right_bumper)) {
            stopperCpos = 0.5;
        }
        if (ALIGN_SHOOT) {
            if (shooterSS.atTarget()) {
                gamepad1.rumble(0.8, 0.8, 1000);
            } else {
                gamepad1.rumble(0, 0, 100);
            }
            if (turretOn) turretSS.align();
            if(shooterOn) shooterSS.align();
            indexerOn = true;
        } else if (RESET_SHOOTER_TURRET) {
            shooterSS.reset();
            turretSS.reset();
            stopperCpos = 0.5;
        }
        if (RESET_INTAKE) {
            pivotCpos = 0;
            indexer.setPower(0);
            intake.setPower(0);
            shooterSS.reset();
        }
        // fixes
        if (gamepad1.dpadRightWasPressed()) {
            turretOffsetR += 1;
            turretOffsetB += 1;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            turretOffsetR -= 1;
            turretOffsetB -= 1;
        }
        if (gamepad1.dpadDownWasReleased()) {
            if (redSide) follower.setPose(new Pose(126, 119, Math.PI - Math.toRadians(144)));
            else follower.setPose(new Pose(18, 119, Math.toRadians(144)));
        }
        // shooter code
        shooterSS.update(follower);
        // turret code
        if (turretSS.getTarget() == 0 && turretTpos != 0) turretSS.setTarget(turretTpos);
        turretSS.update(follower);
        follower.update();
        // telemetry
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addData("loop times", loopTime.milliseconds());
        telemetryM.addData(true, "pivot", pivot.getPosition());
        telemetryM.addData(true, "hood", hood.getPosition());
        telemetryM.addData(true, "indexer", indexer.getPower());
        telemetryM.addData(true, "led", led.getPosition());
        telemetryM.addData(true, "strips", strips.getPosition());
        telemetryM.addData(true, "gameTimer", gameTimer.getElapsedTimeSeconds());
        telemetryM.addLine(true, turretSS.telemetry());
        telemetryM.addLine(true, shooterSS.telemetry());
        telemetryM.addData(true, "indexerOn", indexerOn);
        telemetryM.addData(true, "redSide", redSide);
        telemetryM.addData(true, "beam breaks", !beams.getState());
        telemetryM.update();
        loopTime.reset();
    }

    public static InterpLUT getShooterLUT() {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(26, 1750); // 50
        lut.add(30, 1800); // 25
        lut.add(35, 1825); // 50
        lut.add(40, 1875); // 25
        lut.add(45, 2000); // 75
        lut.add(50, 1975); // 15
        lut.add(55, 1990); // 25
        lut.add(60, 2015); // 175
        lut.add(65, 2053);
        lut.add(70, 2100);
        lut.add(75, 2140);
        // finish
        lut.createLUT();
        return lut;
    }
    public static InterpLUT getHoodLut() {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(26, 0.0);
        lut.add(30, 0.0);
        lut.add(35, 0.2);
        lut.add(40, 0.2);
        lut.add(45, 0.15);
        lut.add(50, 0.23);
        lut.add(55, 0.27);
        lut.add(60, 0.27);
        lut.add(65, 0.3);
        lut.add(70, 0.3);
        lut.add(75, 0.33);
        // finish
        lut.createLUT();
        return lut;
    }
}
