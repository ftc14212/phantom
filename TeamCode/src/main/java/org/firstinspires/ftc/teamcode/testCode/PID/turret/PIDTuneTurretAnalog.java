package org.firstinspires.ftc.teamcode.testCode.PID.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

@Config
@Configurable
@Disabled
@Autonomous(name = "PID Tune Turret Analog", group = "test_ftc14212")
public class PIDTuneTurretAnalog extends OpMode {

    private CombinedCRServo turret;
    private AnalogInput elc;
    private PIDController controller;

    public static double P = 0.00045;
    public static double I = 0;
    public static double D = 0.0002;
    public static double F = 0.02;

    public static double TARGET = 0;

    // Analog encoder settings
    public static double MAX_VOLTAGE = 3.3;

    // Turret zero offset (APPLIED IN TURRET SPACE)
    public static double OFFSET_DEG = -250;

    // Gear ratio: turret / encoder
    public static double GEAR_RATIO = 53.0 / 92.0;

    // Multi-turn tracking
    private double lastEncoderDeg = 0;
    private int turnCount = 0;
    private boolean firstLoop = true;
    public static int turretMaxDeg = 500;

    @Override
    public void init() {
        controller = new PIDController(Math.sqrt(P), I, D);

        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1"));
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2"));
        turret = new CombinedCRServo(turret1, turret2);

        elc = hardwareMap.get(AnalogInput.class, "elc");
        telemetry.addLine("PID Tune Turret Analog");
        telemetry.update();
    }

    @Override
    public void loop() {
        controller.setPID(Math.sqrt(P), I, D);

        /* =============================
           Absolute encoder (-180 to +180)
           ============================= */
        double voltage = elc.getVoltage();
        double encoderDeg = (voltage / MAX_VOLTAGE) * 360.0;

        // FIX: center angle to -180 .. +180 (NO 0–360 wrap)
        encoderDeg = ((encoderDeg + turretMaxDeg)) - (double) turretMaxDeg /2;

        /* =============================
           Multi-turn unwrap
           ============================= */
        if (firstLoop) {
            lastEncoderDeg = encoderDeg;
            turnCount = 0; // FIX
            firstLoop = false;
        }

        double delta = encoderDeg - lastEncoderDeg;

        if (delta > 180) {
            turnCount--;
        } else if (delta < -180) {
            turnCount++;
        }

        lastEncoderDeg = encoderDeg;

        double encoderMultiTurnDeg = encoderDeg + turnCount * 360.0;

        /* =============================
           Apply gear ratio + offset
           ============================= */
        double turretCpos =
                encoderMultiTurnDeg * GEAR_RATIO + OFFSET_DEG; // FIX

        /* =============================
           PID
           ============================= */
        double pid = controller.calculate(turretCpos, TARGET);
        double rawPower = pid + F;

        rawPower = Math.max(-1, Math.min(1, rawPower));
        turret.setPower(-rawPower);

        /* =============================
           Telemetry
           ============================= */
        telemetry.addData("Voltage", voltage);
        telemetry.addData("Encoder Deg", encoderDeg);
        telemetry.addData("Turns", turnCount);
        telemetry.addData("Encoder MT Deg", encoderMultiTurnDeg);
        telemetry.addData("Turret Deg", turretCpos);
        telemetry.addData("Target", TARGET);
        telemetry.addData("Power", rawPower);
        telemetry.addData("Error", TARGET - turretCpos);
        telemetry.update();
    }
}
