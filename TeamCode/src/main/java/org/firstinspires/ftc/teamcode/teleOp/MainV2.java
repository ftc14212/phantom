/***
 * MAIN V2
 * @author David Grieas - 14212 MetroBotics
 * coding for our V2 Phantom - feb 28th
 * started coding at 1/26/25  @  11:54 am
***/
package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.Prism.Color;
import org.firstinspires.ftc.teamcode.utils.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utils.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name="Main v2", group=".ftc14212")
public class MainV2 extends OpMode {
    /**
     * MAIN V2 BY DAVID
     * @author David Grieas - 14212 MetroBotics
    **/
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 0;
    public static double indexerCpos = 0;
    public static double ascendCpos = 0;
    public static double ledCpos = 0.611;
    public static double stripsCpos = 0.611;
    public static double turretTpos = 0;
    public static double shooterVelo = 0;
    public double turretCpos;
    // misc
    private double wheelSpeed = 1;
    public static boolean turretOn = true;
    boolean indexerOn = true;
    // timers
    ElapsedTime loopTime;
    // odometry
    public static boolean odoDrive = true;
    // config stuff
    public static boolean redSide = false;
    public static boolean debugMode = true;
    public static double turretOffset = 0;
    public static double backSpin = 0;
    public static double shooterOffset = -18;
    private final Prompter prompter = new Prompter(this);
    // hardware
    private TelemetryM telemetryM;
    private Follower follower;
    DigitalChannel beams;
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
    CachingDcMotorEx shooterL; // 6000 rpm
    CachingDcMotorEx shooterR; // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm
    CachingDcMotorEx indexer; // 1620 rpm
    // servos
    private static CachingServo pivot; // 1x axon max
    private static CachingServo stopper; // 1x axon mini
    private static CombinedServo hood; // 2x axon mini
    private static CachingServo led; // 2x gobilda led lights RGB
    private static CachingServo strips; // 4x gobilda strip RGB lights
    private static CombinedCRServo turret; // 2x axon mini
    private static CombinedCRServo ascend; // 4x axon max
    // random
    // GoBildaPrismDriver strips;
    // pids
    PIDController turretPID;
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
        turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        // Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        // limelight3A.setPollRateHz(50);
        LynxUtils.initLynx(hardwareMap);
        beams = hardwareMap.get(DigitalChannel.class, "bb");
        // strips = hardwareMap.get(GoBildaPrismDriver.class,"strips");
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
        pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        CachingServo hoodR = new CachingServo(hardwareMap.get(Servo.class, "hoodR")); // 1x axon mini
        CachingServo hoodL = new CachingServo(hardwareMap.get(Servo.class, "hoodL")); // 1x axon mini
        hood = new CombinedServo(hoodR, hoodL); // 2x axon minis
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon mini
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon mini
        turret = new CombinedCRServo(turret1, turret2); // 2x axon minis
        CachingCRServo aR1 = new CachingCRServo(hardwareMap.get(CRServo.class, "aR1")); // 1x axon max
        CachingCRServo aR2 = new CachingCRServo(hardwareMap.get(CRServo.class, "aR2")); // 1x axon max
        CachingCRServo aL1 = new CachingCRServo(hardwareMap.get(CRServo.class, "aL1")); // 1x axon max
        CachingCRServo aL2 = new CachingCRServo(hardwareMap.get(CRServo.class, "aL2")); // 1x axon max
        ascend = new CombinedCRServo(aR1, aR2, aL1, aL2); // 4x axon maxs
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        strips = new CachingServo(hardwareMap.get(Servo.class, "strips")); // 4x gobilda strip RGB lights
        stopper = new CachingServo(hardwareMap.get(Servo.class, "stopper")); // 1x axon mini
        // limits
        hood.scaleRange(0, 0.38);
        pivot.scaleRange(0, 0.4);
        // turn on motor
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // do the strips
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
        pinpoint.recalibrateIMU();
        if (MainV1E.lastAutoPos != null) follower.setStartingPose(new Pose(MainV1E.lastAutoPos.getX(), MainV1E.lastAutoPos.getY(), MainV1E.lastAutoPos.getHeading()));
        MainV1E.lastAutoPos = null;
        if (MainV1E.lastTurretPos != -999) turretCpos = MainV1E.lastTurretPos;
        else indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MainV1E.lastTurretPos = -999;
        // subsystems
        turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, indexer, PIDTuneTurret.TPR, PIDTuneTurret.ratio, turretOffset, MainV1E.lastTurretPos);
        shooterSS = new ShooterSS(shooterPID, shooterR, shooterL, hoodR, hoodL);
        shooterSS.setPoses(getShooterLUT(), 15.1, 134.9, getHoodLut(), 15.1, 134.9);
        // misc
        loopTime = new ElapsedTime();
        follower.update();
        beams.setMode(DigitalChannel.Mode.INPUT);
        // reset
        loopTime.reset();
    }

    public void onPromptsComplete() {
        MainV1E.Alliance alliance = prompter.get("alliance");
        MainV1E.StartPos startPos = prompter.get("start_pos");
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(87.5, 8.3, Math.toRadians(90)));
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(new Pose(56.5, 8.3, Math.toRadians(90)));
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(126, 8.3, Math.PI - Math.toRadians(144)));
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
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // poses
        Pose bluePos = new Pose(11, 137, 0);
        Pose redPos = new Pose(133, 137, 0);
        // debugs
        telemetryM.setDebug(debugMode);
        turretPID.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        turretSS.updatePID(turretPID, PIDTuneTurret.F);
        shooterSS.updatePID(shooterPID);
        shooterSS.setShooterOffset(shooterOffset);
        shooterSS.setPoses(bluePos, redPos);
        turretSS.setPoses(bluePos, redPos);
        // vars
        turretCpos = (-indexer.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
        // status
        boolean INTAKE = gamepad1.left_trigger > 0.1;
        boolean OUTTAKE = gamepad1.right_trigger > 0.1;
        boolean FEED = gamepad1.right_bumper;
        boolean ALIGN_SHOOT = gamepad1.left_bumper;
        boolean RESET_SHOOTER_TURRET = !currentGamepad1.left_bumper && previousGamepad1.left_bumper;
        boolean RESET_INTAKE = (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) || gamepad1.dpadRightWasReleased() || (!(currentGamepad1.right_trigger > 0.1) && previousGamepad1.right_trigger > 0.1) || (!(currentGamepad1.left_trigger > 0.1) && previousGamepad1.left_trigger > 0.1);
        // gamepad stuff
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        // servos
        pivot.setPosition(pivotCpos);
        hood.setPosition(hoodCpos);
        indexer.setPower(indexerCpos);
        led.setPosition(ledCpos);
        strips.setPosition(stripsCpos);
        if(!turretOn) turretSS.turretOn(false);
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
            pivotCpos = 0.75;
            if (indexerOn) indexerCpos = 1;
            intake.setPower(1);
            shooterVelo = backSpin;
            if (!beams.getState()) indexerCpos = 0;
        }
        if (OUTTAKE) {
            indexerOn = true;
            pivotCpos = 0.75;
            indexerCpos = -1;
            intake.setPower(-1);
        }
        if (FEED) {
            indexerOn = true;
            pivotCpos = 0.45;
            indexerCpos = 1;
            intake.setPower(1);
        }
        if (ALIGN_SHOOT) {
            if (shooterR.getVelocity() >= shooterVelo - 100) {
                gamepad1.rumble(0.8, 0.8, 1000);
                ledCpos = 1;
            }
            else {
                gamepad1.rumble(0, 0, 100);
                ledCpos = 0.388;
            }
            shooterSS.shooterOn(true);
            turretSS.turretOn(true);
            indexerOn = true;
        } else if (RESET_SHOOTER_TURRET) {
            shooterSS.shooterOn(false);
            turretSS.turretOn(false);
            ledCpos = 0.611;
        }
        if (RESET_INTAKE) {
            pivotCpos = 0.45;
            indexerCpos = 0;
            intake.setPower(0);
            shooterVelo = 0;
        }
        // shooter code
        shooterSS.alignShooter();
        shooterSS.update(follower);
        // turret code
        if (turretSS.alignTurret() == 0 && turretTpos != 0) turretSS.updateTurretTpos(turretTpos);
        else if (turretSS.alignTurret() != 0) turretTpos = turretSS.turretTpos;
        turretSS.alignTurret();
        turretSS.update(follower);
        follower.update();
        // telemetry
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addData(true, "loop times", loopTime.milliseconds());
        telemetryM.addData(true, "pivot", pivot.getPosition());
        telemetryM.addData(true, "hood", hood.getPosition());
        telemetryM.addData(true, "indexer", indexer.getPower());
        telemetryM.addData(true, "led", led.getPosition());
        telemetryM.addLine(true, turretSS.telemetry());
        telemetryM.addLine(true, shooterSS.telemetry());
        telemetryM.addData(true, "indexerOn", indexerOn);
        telemetryM.addData(true, "redSide", redSide);
        telemetryM.addData(true, "beam breaks", !beams.getState());
        telemetryM.update();
        loopTime.reset();
    }

    public InterpLUT getShooterLUT() {
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
        return lut;
    }
    public InterpLUT getHoodLut() {
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
        return lut;
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
}
