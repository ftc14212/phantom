package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

@Config
@Configurable
@Disabled
@Autonomous(name="PID Tune Turret 2", group="test_ftc14212")
public class PIDTuneTurretAnalog2 extends OpMode {
    private CombinedCRServo turret;
    private AnalogInput elc;
    // private AnalogInput elc;
    private PIDController controller;
    public static double P = 0.00045;
    public static double I = 0;
    public static double D = 0.0002;
    public static double F = 0.02;
    public static double TARGET = 0;
    public static double TPR = 4000; // ticks per revolution
    public static double ratio = (double) 53 / 92;
    public static double elcOffset = 0;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1"));
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2"));
        turret = new CombinedCRServo(turret1, turret2);
        elc = hardwareMap.get(AnalogInput.class, "elc");
        // telemetry
        telemetry.addLine("Use this to tune the turret Analog.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     **/
    @Override
    public void loop() {
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneTurretAnalog2.P), PIDTuneTurretAnalog2.I, PIDTuneTurretAnalog2.D);
        // Get current positions
        double encoderDeg = (elc.getVoltage() / 3.3) * 360.0 + elcOffset;
        double turretOR = (TPR * ratio);
        double turretCpos = (-encoderDeg / turretOR) * 360;
        // Calculate PID
        double pid = controller.calculate(turretCpos, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Apply power
        turret.setPower(-Math.max(-1, Math.min(1, rawPower))); // leader
        // telemetry for debugging
        telemetry.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetry.addData("target", TARGET);
        telemetry.addData("turretCpos", turretCpos);
        telemetry.addData("turretPowerRAW", rawPower);
        telemetry.addData("turretPower", Math.max(-1, Math.min(1, rawPower)));
        telemetry.addData("error", Math.abs(TARGET - turretCpos));
        telemetry.addData("elc raw + offset", encoderDeg);
        telemetry.update();
    }
}