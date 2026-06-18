package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.vars.Tune;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@Configurable
@Disabled
@Autonomous(name="PID Dual Tune Turret", group="test_ftc14212")
public class PIDDualTuneTurret extends OpMode {
    private CombinedCRServo turret;
    private CachingDcMotorEx turretEM;
    // private AnalogInput elc;
    private PIDController farController;
    private PIDController closeController;
    public static Tune.PIDF FAR = new Tune.PIDF(
            0.00041,
            0,
            0.0002,
            0.02);
    public static Tune.PIDF CLOSE = new Tune.PIDF(
            0.0003,
            0,
            0.4,
            0.08);
    public static double tolerance = 30;
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
        farController = new PIDController(Math.sqrt(FAR.P), FAR.I, FAR.D);
        closeController = new PIDController(Math.sqrt(CLOSE.P), CLOSE.I, CLOSE.D);
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
        farController = new PIDController(Math.sqrt(FAR.P), FAR.I, FAR.D);
        closeController = new PIDController(Math.sqrt(CLOSE.P), CLOSE.I, CLOSE.D);
        // Get current positions
        double turretOR = (TPR * ratio);
        double turretCpos = (turretEM.getCurrentPosition() / turretOR) * 360;
        double error = TARGET - turretCpos;
        // Calculate PID
        double pidF = farController.calculate(turretCpos, TARGET);
        double pidC = closeController.calculate(turretCpos, TARGET);
        double ff = 0;
        if (Math.abs(error) > 10) ff = Math.signum(error) * FAR.F;
        double rawPower = Math.abs(error) <= tolerance ? pidC : pidF + ff;
        // Apply power
        turret.setPower(-Math.max(-1, Math.min(1, rawPower))); // leader
        // telemetry for debugging
        telemetry.addData("PIDF Close", "P: " + CLOSE.P + " I: " + CLOSE.I + " D: " + CLOSE.D + " F: " + CLOSE.F);
        telemetry.addData("PIDF Far", "P: " + FAR.P + " I: " + FAR.I + " D: " + FAR.D + " F: " + FAR.F);
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