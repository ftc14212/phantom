package org.firstinspires.ftc.teamcode.testCode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV3;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedDcMotorEx;
import org.firstinspires.ftc.teamcode.utils.CombinedGamepad;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "SOTM (rahhhh)", group = "test_ftc14212")
public class sotmTest extends OpMode {
    private final Prompter prompter = new Prompter(this);
    public boolean debugMode = true;
    public static double turretOffset = 0;
    public static double shooterOffset = 0;
    boolean redSide;
    TurretSS turretSS;
    ShooterSS shooterSS;
    TelemetryM telemetryM;
    Follower follower;
    CachingServo led;
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .onComplete(this::onPromptsComplete);
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        PIDFCoefficients shooterPID = new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F);
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        CachingServo hoodR = new CachingServo(hardwareMap.get(Servo.class, "hoodR")); // 1x axon mini
        CachingServo hoodL = new CachingServo(hardwareMap.get(Servo.class, "hoodL")); // 1x axon mini
        CombinedServo hood = new CombinedServo(hoodR, hoodL); // 2x axon minis
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        DcMotorEx encoder = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        turretSS = new TurretSS(turret, encoder, turretPID, MainV1E.lastTurretPos);
        shooterSS = new ShooterSS(new CombinedDcMotorEx(shooterR, shooterL), hood, led, shooterPID);
        shooterSS.setPoses(MainV3.getShooterLUT(), 15.1, 124.9, MainV3.getHoodLut(), 15.1, 124.9);
        turretSS.setWrapAngles(-170, 170);
        turretSS.update(follower);
        shooterSS.update(follower);
        shooterSS.setLeds(1);
        telemetryM.update();
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
        if (redSide) shooterSS.setLeds(0.278);
        else shooterSS.setLeds(0.611);
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
        Gamepad gamepad1p = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
        CombinedGamepad gamepad1 = new CombinedGamepad(this.gamepad1, gamepad1p);
        // poses
        Pose bluePos = new Pose(6, 138); // BYE
        Pose redPos = new Pose(138, 138);
        follower.setTeleOpDrive(-gamepad1.leftStickY(), -gamepad1.leftStickX(), -gamepad1.rightStickX(), true);
        turretSS.setOffset(turretOffset);;
        shooterSS.setOffset(shooterOffset);
        shooterSS.setPose(bluePos, redPos);
        turretSS.setPoses(bluePos, redPos);

        shooterSS.update(follower);
        turretSS.update(follower);
        if (gamepad1.leftBumper() || gamepad1p.left_bumper) {
            shooterSS.sotm();
            turretSS.sotm();
        } else {
            shooterSS.setLeds(redSide ? 0.278 : 0.611);
            shooterSS.reset();
            turretSS.reset();
        }
        telemetryM.addLine(turretSS.telemetry());
        telemetryM.addLine(shooterSS.telemetry());
        telemetryM.update();
    }
}
