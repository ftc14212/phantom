package org.firstinspires.ftc.teamcode.twyls;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.testCode.PID.shooter.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@Configurable
@Disabled
@TeleOp(name = "measureShooter twyls", group = "test_ftc14212")
public class measureShooterTwyls extends LinearOpMode {
    boolean debugMode = true;
    public static int shooterVelo = 0;
    public static double hoodCpos = 0;
    @Override
    public void runOpMode() {
        // hardware
        PIDController shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        CachingDcMotorEx indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        CachingDcMotorEx intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1620 rpm
        // servos
        CachingServo hood1 = new CachingServo(hardwareMap.get(Servo.class, "hood1")); // 1x axon mini
        CachingServo hood2 = new CachingServo(hardwareMap.get(Servo.class, "hood2")); // 1x axon mini
        CombinedServo hood = new CombinedServo(hood1, hood2); // 2x axon minis
        // reverse
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
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
                shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdkTwyls.P,PIDTuneShooterSdkTwyls.I,PIDTuneShooterSdkTwyls.D,PIDTuneShooterSdkTwyls.F));
                shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTuneShooterSdkTwyls.P,PIDTuneShooterSdkTwyls.I,PIDTuneShooterSdkTwyls.D,PIDTuneShooterSdkTwyls.F));
                indexer.setPower(1);
                intake.setPower(1);
                hood.setPosition(hoodCpos);
                shooterR.setVelocity(shooterVelo); // leader
                shooterL.setVelocity(shooterVelo); // follower
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
}
