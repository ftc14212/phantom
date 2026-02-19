package org.firstinspires.ftc.teamcode.testCode.PID.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name="PID Tune Shooter sdk", group="test_ftc14212")
public class PIDTuneShooterSdk extends OpMode {
    private CachingDcMotorEx shooterR;
    private CachingDcMotorEx shooterL;
    public static double P = 100;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12;
    public static double TARGET = 0; // 3100 max
    TelemetryM telemetryM;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, true);
        // hardware
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR"));
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL"));
        // reverse motor
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        // turn on the motors without the built in controller
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // turn on breaks
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // set the PID values
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I,D,F));
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I,D,F));
        // combine both FTCDashboard and the regular telemetry
        // telemetry
        telemetry.addLine("Use this to tune the turret.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     **/
    @Override
    public void loop() {
        // Get current positions
        double velocityR = shooterR.getVelocity();
        double velocityL = shooterL.getVelocity();
        // Update PID values
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I,D,F));
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P,I,D,F));
        // F = TARGET;
        // Apply power
        shooterR.setVelocity(TARGET); // leader
        shooterL.setVelocity(TARGET); // follower
        // telemetry for debugging
        telemetryM.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetryM.addData("target", TARGET);
        telemetryM.addData("velocityR", velocityR);
        telemetryM.addData("velocityL", velocityL);
        telemetryM.addData("rawVelocity", (velocityL + velocityR)/2);
        telemetryM.addData("errorR", Math.abs(TARGET - velocityR));
        telemetryM.addData("errorL", Math.abs(TARGET - velocityL));
        telemetryM.addData("errorAvg", (Math.abs(TARGET - velocityR) + Math.abs(TARGET - velocityL)) / 2);
        telemetryM.update();
    }
}