package org.firstinspires.ftc.teamcode.testCode.PID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

@Config
@Configurable
@Disabled
@Autonomous(name="PID Tune Servo", group="test_ftc23403")
public class PIDTuneServo extends OpMode {
    private CachingCRServo servo;
    private AnalogInput encoder;
    private PIDController controller;
    public static double P = 0.000044;
    public static double I = 0;
    public static double D = 0.000048;
    public static double TARGET = 175;

    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        servo = new CachingCRServo(hardwareMap.get(CRServo.class, "rr"));
        encoder = hardwareMap.get(AnalogInput.class, "rrA");
        // combine both FTCDashboard and the regular telemetry
        // telemetry
        telemetry.addLine("Use this to tune the servo.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     **/
    @Override
    public void loop() {
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneServo.P), PIDTuneServo.I, PIDTuneServo.D);
        // Get current positions
        double pos = ((encoder.getVoltage() / 3.3)) * 360;
        // Calculate PID
        double pid = controller.calculate(pos, TARGET);
        // Apply power
        servo.setPower(Math.max(-1, Math.min(1, pid)));
        // telemetry for debugging
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D);
        telemetry.addData("target", TARGET);
        telemetry.addData("current pos", pos);
        telemetry.addData("power", servo.getPower());
        telemetry.addData("error", Math.abs(TARGET - pos));
        telemetry.update();
    }
}
