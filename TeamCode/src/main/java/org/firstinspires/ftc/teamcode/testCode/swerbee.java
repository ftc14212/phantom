/***
 * swerbee
 * @author David Grieas - 14212 MetroBotics - for 23403 - C{}de C<>nduct<>rs
 * full swerve drive code in teleOp
 * started coding at 12/2/25  @  7:15 pm
 * finished coding at 12/2/25  @  11:27 pm
***/
package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Disabled
@Config
@Configurable
@TeleOp(name = "swerbee", group = "test_ftc14212")
public class swerbee extends LinearOpMode {
    /**
     * SWERBEE BY DAVID
     * @author David Grieas - 14212 MetroBotics
    **/
    // --------------------
    // Tunables (adjust while testing)
    // --------------------
    public static boolean debugMode = true;

    // Steering PID (per-module instances created below)
    public static double steerP = 0.00002;
    public static double steerI = 0.0;
    public static double steerD = 0.000000004;

    // scale of steering PID output -> CRServo power (-1..1)
    public static double steerOutputScale = 1.0;

    // servo smoothing (0.0 = no movement, 1.0 = instant set)
    public static double servoSmooth = 0.25;

    // overall drive speed scalar
    public static double mSpeed = 1.0;

    // physical geometry - wheelbase (front-back) and trackwidth (left-right) - set to real measurements (any units)
    public static double L = 10.0;
    public static double W = 10.0;

    // analog-to-degree offsets for each module (use your calibrated values)
    public static double rfOffset = 188.0;
    public static double lfOffset = 135.0;
    public static double rrOffset = 115.0;
    public static double lrOffset = 55.0;

    // steering deadband: if angle error (deg) < this, stop driving steering servo
    public static double steerDeadbandDeg = 2.0;

    // If a wheel is physically inverted (driving forward should be negative power), flip driveSign for that module
    // (tweak if a drive motor runs backward)
    public static int rfDriveSign = 1;
    public static int lfDriveSign = 1;
    public static int rrDriveSign = 1;
    public static int lrDriveSign = 1;

    // If a steer CRServo moves opposite to desired PID sign, flip steerSign for that module
    public static int rfSteerSign = 1;
    public static int lfSteerSign = 1;
    public static int rrSteerSign = 1;
    public static int lrSteerSign = 1;

    // --------------------
    // internal objects & state
    // --------------------
    private TelemetryM telemetryM;

    private CachingDcMotorEx leftFront, leftRear, rightFront, rightRear;
    private CachingCRServo lfServo, rfServo, lrServo, rrServo;  // steering CRServos
    private AnalogInput rfA, lfA, rrA, lrA;                     // absolute analog sensors

    // Steering PID controllers (one each)
    private PIDController pidRF = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private PIDController pidLF = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private PIDController pidRR = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private PIDController pidLR = new PIDController(Math.sqrt(steerP), steerI, steerD);

    // Smoothed (current) wheel angle targets we are commanding (deg 0..360)
    private double targetAngleRF = 0;
    private double targetAngleLF = 0;
    private double targetAngleRR = 0;
    private double targetAngleLR = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = new TelemetryM(telemetry, debugMode);

        // --- Hardware mapping (using the exact names you provided) ---
        leftFront  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftRear   = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightRear  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));

        lfServo = new CachingCRServo(hardwareMap.get(CRServo.class, "lf"));
        rfServo = new CachingCRServo(hardwareMap.get(CRServo.class, "rf"));
        lrServo = new CachingCRServo(hardwareMap.get(CRServo.class, "lr"));
        rrServo = new CachingCRServo(hardwareMap.get(CRServo.class, "rr"));

        rfA = hardwareMap.get(AnalogInput.class, "rfA");
        lfA = hardwareMap.get(AnalogInput.class, "lfA");
        rrA = hardwareMap.get(AnalogInput.class, "rrA");
        lrA = hardwareMap.get(AnalogInput.class, "lrA");

        // Keep motor directions similar to your previous setup (change if needed)
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset PIDs
        pidRF.reset(); pidLF.reset(); pidRR.reset(); pidLR.reset();

        // initialize servo powers to zero
        lfServo.setPower(0);
        rfServo.setPower(0);
        lrServo.setPower(0);
        rrServo.setPower(0);

        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "Swerve INIT DONE");
        telemetryM.update();



        waitForStart();

        // Precompute R
        double R = Math.hypot(L, W);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Gamepad gamepad1p = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
                // read raw analog volts -> degrees
                double rawRfDeg = analogToDeg(rfA.getVoltage());
                double rawLfDeg = analogToDeg(lfA.getVoltage());
                double rawRrDeg = analogToDeg(rrA.getVoltage());
                double rawLrDeg = analogToDeg(lrA.getVoltage());

                // apply offsets & wrap to 0..360
                double measuredRF = wrapDeg(rawRfDeg - rfOffset);
                double measuredLF = wrapDeg(rawLfDeg - lfOffset);
                double measuredRR = wrapDeg(rawRrDeg - rrOffset);
                double measuredLR = wrapDeg(rawLrDeg - lrOffset);

                // driver inputs
                double forward = -gamepad1p.left_stick_y; // forward positive
                double strafe  =  gamepad1p.left_stick_x; // right positive
                double rotate  =  gamepad1p.right_stick_x; // clockwise positive

                // optional small deadband
                forward = applyDeadband(forward, 0.03);
                strafe  = applyDeadband(strafe, 0.03);
                rotate  = applyDeadband(rotate, 0.03);

                // swerve kinematics
                double A = strafe - rotate * (L / R);
                double B = strafe + rotate * (L / R);
                double C = forward - rotate * (W / R);
                double D = forward + rotate * (W / R);

                // compute desired wheel speeds (magnitudes)
                double speedLF = Math.hypot(B, D); // front-left
                double speedRF = Math.hypot(B, C); // front-right
                double speedLR = Math.hypot(A, D); // rear-left
                double speedRR = Math.hypot(A, C); // rear-right

                // compute desired wheel angles (0 = forward)
                double angleLF = wrapDeg(Math.toDegrees(Math.atan2(B, D)));
                double angleRF = wrapDeg(Math.toDegrees(Math.atan2(B, C)));
                double angleLR = wrapDeg(Math.toDegrees(Math.atan2(A, D)));
                double angleRR = wrapDeg(Math.toDegrees(Math.atan2(A, C)));

                // If robot commanded to do nothing, zero everything
                boolean zero = (Math.abs(forward) < 1e-6 && Math.abs(strafe) < 1e-6 && Math.abs(rotate) < 1e-6);

                // Normalize wheel speeds to <= 1
                double max = Math.max(Math.max(speedLF, speedRF), Math.max(speedLR, speedRR));
                if (max > 1.0) {
                    speedLF /= max; speedRF /= max; speedLR /= max; speedRR /= max;
                }

                // Apply overall speed scaler
                speedLF *= mSpeed; speedRF *= mSpeed; speedLR *= mSpeed; speedRR *= mSpeed;

                // Optimize each module: choose shortest steering rotation; if >90°, flip angle 180 and invert speed
                SwerveModuleState optLF = optimize(angleLF, speedLF, measuredLF);
                SwerveModuleState optRF = optimize(angleRF, speedRF, measuredRF);
                SwerveModuleState optLR = optimize(angleLR, speedLR, measuredLR);
                SwerveModuleState optRR = optimize(angleRR, speedRR, measuredRR);

                // Smooth target angles to avoid overshoot / continuous spinning
                targetAngleLF = smoothAngleTowards(targetAngleLF, optLF.angle, servoSmooth);
                targetAngleRF = smoothAngleTowards(targetAngleRF, optRF.angle, servoSmooth);
                targetAngleLR = smoothAngleTowards(targetAngleLR, optLR.angle, servoSmooth);
                targetAngleRR = smoothAngleTowards(targetAngleRR, optRR.angle, servoSmooth);

                // Steering PID outputs (drive CRServo)
                double errLF = Math.abs(angleError(targetAngleLF, measuredLF));
                double errRF = Math.abs(angleError(targetAngleRF, measuredRF));
                double errLR = Math.abs(angleError(targetAngleLR, measuredLR));
                double errRR = Math.abs(angleError(targetAngleRR, measuredRR));

                double pidOutLF = (errLF > steerDeadbandDeg ? pidLF.calculate(measuredLF, targetAngleLF) : 0.0) * steerOutputScale * lfSteerSign;
                double pidOutRF = (errRF > steerDeadbandDeg ? pidRF.calculate(measuredRF, targetAngleRF) : 0.0) * steerOutputScale * rfSteerSign;
                double pidOutLR = (errLR > steerDeadbandDeg ? pidLR.calculate(measuredLR, targetAngleLR) : 0.0) * steerOutputScale * lrSteerSign;
                double pidOutRR = (errRR > steerDeadbandDeg ? pidRR.calculate(measuredRR, targetAngleRR) : 0.0) * steerOutputScale * rrSteerSign;

                // Clamp servo outputs (-1..1)
                pidOutLF = clamp(pidOutLF, -1, 1);
                pidOutRF = clamp(pidOutRF, -1, 1);
                pidOutLR = clamp(pidOutLR, -1, 1);
                pidOutRR = clamp(pidOutRR, -1, 1);

                // Set CRServo powers for steering
                lfServo.setPower(pidOutLF);
                rfServo.setPower(pidOutRF);
                lrServo.setPower(pidOutLR);
                rrServo.setPower(pidOutRR);

                // Drive motors: set power to desired speeds (apply drive sign if necessary)
                leftFront.setPower(zero ? 0.0 : optLF.speed * lfDriveSign);
                rightFront.setPower(zero ? 0.0 : optRF.speed * rfDriveSign);
                leftRear.setPower(zero ? 0.0 : optLR.speed * lrDriveSign);
                rightRear.setPower(zero ? 0.0 : optRR.speed * rrDriveSign);

                // Telemetry for debugging
                telemetryM.addLine("SWERVE STATUS");
                telemetryM.addLine(true, "Measured (deg):");
                telemetryM.addData(true, "LF meas", measuredLF);
                telemetryM.addData(true, "RF meas", measuredRF);
                telemetryM.addData(true, "LR meas", measuredLR);
                telemetryM.addData(true, "RR meas", measuredRR);

                telemetryM.addLine(true, "\nTargets (smoothed):");
                telemetryM.addData(true, "LF tgt", targetAngleLF);
                telemetryM.addData(true, "RF tgt", targetAngleRF);
                telemetryM.addData(true, "LR tgt", targetAngleLR);
                telemetryM.addData(true, "RR tgt", targetAngleRR);

                telemetryM.addLine(true, "\nModule states:");
                telemetryM.addData(true, "LF state", stateToString(optLF));
                telemetryM.addData(true, "RF state", stateToString(optRF));
                telemetryM.addData(true, "LR state", stateToString(optLR));
                telemetryM.addData(true, "RR state", stateToString(optRR));

                telemetryM.addLine(true, "\nPID outs:");
                telemetryM.addData(true, "LF pid", pidOutLF);
                telemetryM.addData(true, "RF pid", pidOutRF);
                telemetryM.addData(true, "LR pid", pidOutLR);
                telemetryM.addData(true, "RR pid", pidOutRR);

                telemetryM.update();
            }
        }
    }

    // --------------------
    // Helper classes & functions
    // --------------------
    private static class SwerveModuleState {
        double angle; // 0..360
        double speed; // -1..1 (negative = wheel reversed)
        SwerveModuleState(double a, double s) { angle = a; speed = s; }
    }

    /**
     * Optimize the desired angle+speed relative to currentAngle to pick the shortest steering change.
     * If rotating more than 90 degrees, flip angle by 180 and invert speed (so wheel reverses instead of long-turn).
     * Returns a state with angle in 0..360 and speed scaled.
     */
    private SwerveModuleState optimize(double desiredAngle, double desiredSpeed, double currentAngle) {
        desiredAngle = wrapDeg(desiredAngle);
        currentAngle = wrapDeg(currentAngle);

        double delta = angleError(desiredAngle, currentAngle); // signed -180..180

        // if greater than 90 degrees, flip
        if (Math.abs(delta) > 90.0) {
            desiredAngle = wrapDeg(desiredAngle + 180.0);
            desiredSpeed = -desiredSpeed;
        }

        // ensure angle normalized
        desiredAngle = wrapDeg(desiredAngle);

        return new SwerveModuleState(desiredAngle, desiredSpeed);
    }

    /**
     * Convert analog voltage (0..3.3 V typical) to degrees (0..360).
     * If your analog sensor uses a different top voltage, change the 3.3 factor.
     */
    private static double analogToDeg(double voltage) {
        // avoid division by zero in weird cases
        double v = Math.max(0.0, Math.min(voltage, 3.3));
        return (v / 3.3) * 360.0;
    }

    /**
     * Wrap degrees to 0..360
     */
    private static double wrapDeg(double deg) {
        deg %= 360.0;
        if (deg < 0.0) deg += 360.0;
        return deg;
    }

    /**
     * Shortest signed difference: target - current in range [-180, 180]
     */
    private static double angleError(double target, double current) {
        double err = target - current;
        err = (err + 180.0) % 360.0 - 180.0;
        return err;
    }

    /**
     * Smoothly move current toward target across wrap boundary, returns new current
     */
    private static double smoothAngleTowards(double current, double target, double smoothing) {
        current = wrapDeg(current);
        target = wrapDeg(target);
        double err = angleError(target, current);
        double delta = err * Math.max(0.0, Math.min(1.0, smoothing));
        double next = current + delta;
        return wrapDeg(next);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private static String stateToString(SwerveModuleState s) {
        return String.format("angle=%.1f speed=%.3f", s.angle, s.speed);
    }
}
