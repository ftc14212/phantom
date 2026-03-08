package org.firstinspires.ftc.teamcode.testCode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV2;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "measureShooter", group = "test_ftc14212")
public class measureShooter extends LinearOpMode {
    boolean debugMode = true;
    public static boolean turnOn = false;
    public static int shooterVelo = 0;
    public static double hoodCpos = 0;
    public static double turretTpos = 0;
    public static double indexerSpeed = 1;
    @Override
    public void runOpMode() {
        // hardware
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Follower follower = Constants.createFollower(hardwareMap);
        // motors
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        CachingDcMotorEx indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1620 rpm
        // servos
        CachingServo hoodR = new CachingServo(hardwareMap.get(Servo.class, "hoodR")); // 1x axon mini
        CachingServo hoodL = new CachingServo(hardwareMap.get(Servo.class, "hoodL")); // 1x axon mini
        CombinedServo hood = new CombinedServo(hoodR, hoodL); // 2x axon minis
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon mini
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon mini
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon minis
        // limits
        hood.scaleRange(0, 0.38);
        // reverse
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        TurretSS turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, indexer, PIDTuneTurret.TPR, PIDTuneTurret.ratio, MainV2.turretOffsetB, MainV1E.lastTurretPos);
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                turretSS.updatePID(turretPID, PIDTuneTurret.F);
                turretSS.setTurretOffset(MainV2.turretOffsetB);
                turretSS.updateTurretTpos(turretTpos);
                turretSS.update(follower);
                shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
                shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
                indexer.setPower(turnOn ? indexerSpeed : 0);
                hood.setPosition(hoodCpos);
                shooterR.setVelocity(turnOn ? shooterVelo : 0); // leader
                shooterL.setVelocity(turnOn ? shooterVelo : 0); // follower
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
}
