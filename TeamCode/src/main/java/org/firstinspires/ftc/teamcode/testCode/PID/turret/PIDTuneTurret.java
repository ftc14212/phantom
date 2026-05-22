package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.vars.Tune;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name="PID Tune Turret", group="test_ftc14212")
public class PIDTuneTurret extends OpMode {
    private CombinedCRServo turret;
    private CachingDcMotorEx turretEM;
    private PIDController controller;
    private TrapezoidalMotionProfile profile;
    public static Tune.PIDF pidf = new Tune.PIDF(
            0.02,
            0,
            0.0001,
            0.09
    );
    public static double TARGET = 0;
    public static int TPR = 4000;
    public static double ratio = (double) 120 / 32;
    // motion profile constraints
    public static double maxVel = 180;
    public static double maxAccel = 360;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry().getWrapper());
        controller = new PIDController(pidf.P, pidf.I, pidf.D);
        // hardware
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1"));
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2"));
        turret = new CombinedCRServo(turret1, turret2);
        turretEM = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer"));
        // encoder
        turretEM.setDirection(DcMotorEx.Direction.REVERSE);
        turretEM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // motion profile
        profile = new TrapezoidalMotionProfile(maxVel, maxAccel);
        profile.reset(0);
        telemetry.addLine("Use this to tune the turret.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // update PID values live
        controller.setPID(pidf.P, pidf.I, pidf.D);
        // update profile constraints live
        profile.setConstraints(maxVel, maxAccel);
        // turret position
        double turretOR = (TPR * ratio);
        double turretCpos = (turretEM.getCurrentPosition() / turretOR) * 360;
        // profiled target
        double profiledTarget = profile.update(TARGET);
        // PID
        double pid = controller.calculate(turretCpos, profiledTarget);
        // feedforward
        double error = profiledTarget - turretCpos;
        double ff = 0;
        if (Math.abs(error) > 2) ff = Math.signum(error) * pidf.F;
        // total power
        double rawPower = pid + ff;
        rawPower = Math.max(-1, Math.min(1, rawPower));
        // apply power
        turret.setPower(rawPower);
        // telemetry
        telemetry.addData("PIDF", "P: " + pidf.P + " I: " + pidf.I + " D: " + pidf.D + " F: " + pidf.F);
        telemetry.addData("TARGET", TARGET);
        telemetry.addData("PROFILED TARGET", profiledTarget);
        telemetry.addData("CURRENT POSITION", turretCpos);
        telemetry.addData("PROFILE VELOCITY", profile.getVelocity());
        telemetry.addData("PID", pid);
        telemetry.addData("FF", ff);
        telemetry.addData("RAW POWER", rawPower);
        telemetry.addData("ERROR", error);
        telemetry.addData("CURRENT TICKS", turretEM.getCurrentPosition());
        telemetry.update();
    }
}