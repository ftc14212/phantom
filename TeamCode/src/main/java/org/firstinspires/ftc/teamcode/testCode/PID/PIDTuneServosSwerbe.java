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
@Autonomous(name="PID Tune Servos", group="test_ftc23403")
public class PIDTuneServosSwerbe extends OpMode {
    CachingCRServo lf;
    CachingCRServo rf;
    CachingCRServo lr;
    CachingCRServo rr;
    AnalogInput rfA;
    AnalogInput lfA;
    AnalogInput rrA;
    AnalogInput lrA;
    private PIDController controller;
    public static double P = 0.00002;
    public static double I = 0;
    public static double D = 0.000000004;
    public static double frT = 175;
    public static double flT = 175;
    public static double blT = 175;
    public static double brT = 175;

    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        // servos
        lf = new CachingCRServo(hardwareMap.get(CRServo.class, "lf"));
        rf = new CachingCRServo(hardwareMap.get(CRServo.class, "rf"));
        lr = new CachingCRServo(hardwareMap.get(CRServo.class, "lr"));
        rr = new CachingCRServo(hardwareMap.get(CRServo.class, "rr"));
        // analogs
        rfA = hardwareMap.get(AnalogInput.class, "rfA");
        lfA = hardwareMap.get(AnalogInput.class, "lfA");
        rrA = hardwareMap.get(AnalogInput.class, "rrA");
        lrA = hardwareMap.get(AnalogInput.class, "lrA");
        // combine both FTCDashboard and the regular telemetry
        // telemetry
        telemetry.addLine("Use this to tune the servos.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     **/
    @Override
    public void loop() {
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneServosSwerbe.P), PIDTuneServosSwerbe.I, PIDTuneServosSwerbe.D);
        // Get current positions
        double lfPos = ((lfA.getVoltage() / 3.3)) * 360;
        double rfPos = ((rfA.getVoltage() / 3.3)) * 360;
        double rrPos = ((rrA.getVoltage() / 3.3)) * 360;
        double lrPos = ((lrA.getVoltage() / 3.3)) * 360;
        // Apply power
        lf.setPower(Math.max(-1, Math.min(1, controller.calculate(lfPos, flT))));
        lr.setPower(Math.max(-1, Math.min(1, controller.calculate(lrPos, blT))));
        rr.setPower(Math.max(-1, Math.min(1, controller.calculate(rrPos, brT))));
        rf.setPower(Math.max(-1, Math.min(1, controller.calculate(rfPos, frT))));
        // telemetry for debugging
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D);
        telemetry.addData("\nflT", flT);
        telemetry.addData("frT", frT);
        telemetry.addData("blT", blT);
        telemetry.addData("brT", brT);
        telemetry.addData("\nlfPos", lfPos);
        telemetry.addData("rfPos", rfPos);
        telemetry.addData("lrPos", lrPos);
        telemetry.addData("rrPos", rrPos);
        telemetry.addData("\nlf Power", lf.getPower());
        telemetry.addData("rf Power", rf.getPower());
        telemetry.addData("lr Power", lr.getPower());
        telemetry.addData("rr Power", rr.getPower());
        telemetry.addData("\nlf error", Math.abs(flT - lfPos));
        telemetry.addData("rf error", Math.abs(frT - rfPos));
        telemetry.addData("lr error", Math.abs(blT - lrPos));
        telemetry.addData("rr error", Math.abs(brT - rrPos));
        telemetry.update();
    }
}
