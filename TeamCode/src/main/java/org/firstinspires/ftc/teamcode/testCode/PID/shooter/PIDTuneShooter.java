package org.firstinspires.ftc.teamcode.testCode.PID.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@Configurable
@Disabled
@Autonomous(name="PID Tune Shooter", group="test_ftc14212")
public class PIDTuneShooter extends OpMode {
    private CachingDcMotorEx shooterR;
    private CachingDcMotorEx shooterL;
    private PIDController controller;
    public static double P = 8;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double TARGET = 0;
    TelemetryM telemetryM;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        telemetryM = new TelemetryM(telemetry, true);
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR"));
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL"));
        // reverse motor
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        // turn on the motors without the built in controller
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        controller.setPID(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        F = TARGET;
        // Calculate PID only on one motor (leader)
        double pid = controller.calculate(velocityR, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Apply power
        shooterR.setVelocity(rawPower); // leader
        shooterL.setVelocity(rawPower); // follower
        // telemetry for debugging
        telemetryM.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetryM.addData("target", TARGET);
        telemetryM.addData("velocityR", velocityR);
        telemetryM.addData("velocityL", velocityL);
        telemetryM.addData("rawVelocity", rawPower);
        telemetryM.addData("errorR", Math.abs(TARGET - velocityR));
        telemetryM.addData("errorL", Math.abs(TARGET - velocityL));
        telemetryM.addData("errorAvg", (Math.abs(TARGET - velocityR) + Math.abs(TARGET - velocityL)) / 2);
        telemetryM.update();
    }
}