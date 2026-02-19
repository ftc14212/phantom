package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name="PID Tune Turret", group="test_ftc14212")
public class PIDTuneTurret extends OpMode {
    private CombinedCRServo turret;
    private CachingDcMotorEx turretEM;
    // private AnalogInput elc;
    private PIDController controller;
    public static double P = 0.0001;
    public static double I = 0;
    public static double D = 0.0005;
    public static double F = 0;
    public static double TARGET = 0;
    public static int TPR = 4000; // ticks per revolution
    public static double ratio = (double) 120 / 32;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1"));
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2"));
        turret = new CombinedCRServo(turret1, turret2);
        turretEM = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer"));
        // reset encoders
        turretEM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on the motors without the built in controller
        turretEM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        // Get current positions
        double turretOR = (TPR * ratio);
        double turretCpos = (turretEM.getCurrentPosition() / turretOR) * 360;
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
        telemetry.addData("current ticks", turretEM.getCurrentPosition());
        telemetry.addData("target ticks", TARGET*TPR);
        telemetry.update();
    }
}