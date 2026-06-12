package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooterSdk;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDDualTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
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
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        CachingDcMotorEx indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1620 rpm
        CachingDcMotorEx intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm --> 460 rpm
        // servos
        CachingServo hoodR = new CachingServo(hardwareMap.get(Servo.class, "hoodR")); // 1x axon mini
        CachingServo hoodL = new CachingServo(hardwareMap.get(Servo.class, "hoodL")); // 1x axon mini
        CombinedServo hood = new CombinedServo(hoodR, hoodL); // 2x axon minis
        CachingServo stopper = new CachingServo(hardwareMap.get(Servo.class, "stopper")); // 1x axon mini
        CachingServo pivot = new CachingServo(hardwareMap.get(Servo.class, "pivot")); // 1x axon max
        // limits
        stopper.scaleRange(0.42, 1);
        pivot.scaleRange(0, 0.375);
        hood.scaleRange(0, 0.37);
        // reverse`
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(gamepad1.left_bumper) stopper.setPosition(0);
                else stopper.setPosition(0.5);
                shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
                shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdk.P,PIDTuneShooterSdk.I,PIDTuneShooterSdk.D,PIDTuneShooterSdk.F));
                indexer.setPower(turnOn ? indexerSpeed : 0);
                intake.setPower(turnOn ? indexerSpeed : 0);
                hood.setPosition(hoodCpos);
                pivot.setPosition(0.06);
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
