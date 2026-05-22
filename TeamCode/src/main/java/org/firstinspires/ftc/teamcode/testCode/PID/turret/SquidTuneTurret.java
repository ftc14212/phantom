package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.SquIDFController;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.vars.Tune;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name="Squid Tune Turret", group="test_ftc14212")
public class SquidTuneTurret extends OpMode {
    private CombinedCRServo turret;
    private CachingDcMotorEx turretEM;
    // private AnalogInput elc;
    private SquIDFController controller;
    public static Tune.PIDF pidf = new Tune.PIDF(
            0.000025,
            0,
            0.01,
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
        controller = new SquIDFController(pidf.P, pidf.I, pidf.D, pidf.F);
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
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        controller = new SquIDFController(pidf.P, pidf.I, pidf.D, pidf.F);
        double currentPos = (turretEM.getCurrentPosition() / (TPR * ratio)) * 360.0;
        double error = TARGET - currentPos;
        double pid = controller.calculate(currentPos, TARGET);
        if (Double.isNaN(pid)) pid = 0;
        pid = Math.max(-1, Math.min(1, pid));
        if (Math.abs(error) < tolerance) pid = 0;
        turret.setPower(pid);
        // telemetry for debugging
        telemetry.addData("PIDF", "P: " + pidf.P + " I: " + pidf.I + " D: " + pidf.D + " F: " + pidf.F);
        telemetry.addData("target", TARGET);
        telemetry.addData("turretCpos", currentPos);
        telemetry.addData("turretPowerRAW", pid);
        telemetry.addData("turretPower", Math.max(-1, Math.min(1, pid)));
        telemetry.addData("error", Math.abs(error));
        telemetry.addData("current ticks", turretEM.getCurrentPosition());
        telemetry.addData("target ticks", TARGET*TPR);
        telemetry.update();
    }
}
