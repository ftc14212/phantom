package org.firstinspires.ftc.teamcode.twyls;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
@TeleOp(name="Teleop", group = "TYWLS26")
public class twylsOLD extends LinearOpMode {


    private DcMotorEx shooter1, shooter2;
    GoBildaPinpointDriver pinpoint;

    private PanelsTelemetry panelsTelemetry;
    private DcMotorEx intake;
    private DcMotor indexer;
    private CRServo turret;
    private AnalogInput turretPOT;
    public Follower follower;
    private double turretOffset = 5.5;
    private double rawTurretZero = 193.9;
    private double testingRelativeTurretPosition = 15.0;
    private double redShooter = 32;

    // Turret control parameters
    private static final double TURRET_TARGET = 15.0;  // ABSOLUTE target position (15°)
    private static final double TURRET_TOLERANCE = 3.0;  // Within 3° is good enough
    private static final double TURRET_DECEL_ZONE = 10; // Start slowing down 10° before target
    private static final double TURRET_STOP_EARLY = 5;  // Stop 5° before target to account for momentum

    // Velocity tracking for turret
    private double lastTurretAngle = 0.0;
    private double turretVelocity = 0.0;
    private ElapsedTime turretTimer = new ElapsedTime();

    static final double MAX_VOLTAGE = 3.3;
    static final double DEGREES_PER_REV = 360.0;



    private Pose redGoalPs = new Pose (86.2, 134.5, Math.toRadians(redShooter));
    private Pose blueGoalPs = new Pose(34.9, 121.9, Math.toRadians(0));
    Pose blueStart = new Pose(33, 136, Math.toRadians(0));

    private boolean gamepadDown = false;

    double lastAngleDeg = 0.0;
    int turnCount = 0;

    // Call once at init
    void initEncoder(double initialVoltage) {
        lastAngleDeg = voltageToDegrees(initialVoltage);
        turnCount = 0;
    }

    // Call every loop
    public static double mapValue(double originalValue) {
        // Ensure the input value is within the expected bounds (optional, but good practice)
        if (originalValue < 0.0) {
            return 0.0;
        }
        if (originalValue > 3.3) {
            return 1.0;
        }

        // Perform the linear scaling
        return originalValue / 3.3;
    }

    double updateEncoder(double voltage) {
        double currentAngleDeg = voltageToDegrees(voltage);

        double delta = currentAngleDeg - lastAngleDeg;

        // Detect wrap-around
        if (delta > 180.0) {
            // Wrapped backwards (e.g. 5° → 355°)
            turnCount--;
        } else if (delta < -180.0) {
            // Wrapped forwards (e.g. 355° → 5°)
            turnCount++;
        }

        lastAngleDeg = currentAngleDeg;

        // Continuous angle (can grow positive or negative)
        return (turnCount * DEGREES_PER_REV) + currentAngleDeg;
    }

    public double getAbsoluteDeg () {
        return(turretPOT.getVoltage()/MAX_VOLTAGE)*DEGREES_PER_REV;
    }

    public void reset (){
        turnCount = 0;
        lastAngleDeg = getAbsoluteDeg();
    }

    // Utility
    double voltageToDegrees(double voltage) {
        return (voltage / MAX_VOLTAGE) * DEGREES_PER_REV;
    }

    /**
     * SOLUTION 1: Simple predictive stop with deceleration
     * Moves to ABSOLUTE target position (e.g., 15°) from any starting position
     */
    private void controlTurretSimple(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;  // Positive = need to move forward, Negative = need to move backward

        // Check if we've reached the target
        if (Math.abs(error) < TURRET_TOLERANCE) {
            turret.setPower(0);
            telemetry.addData("Turret Status", "AT TARGET");
            return;
        }

        // STOP EARLY to account for momentum
        if (Math.abs(error) < TURRET_STOP_EARLY) {
            turret.setPower(0);
            telemetry.addData("Turret Status", "COASTING TO STOP");
            return;
        }

        // Determine direction and calculate power
        double power;

        if (error < 0) {
            // Need to move forward (increase angle)
            // Since your servo is CC (counter-clockwise) with negative power, use positive power to go forward
            power = 1.0;  // Positive power
        } else {
            // Need to move backward (decrease angle)
            power = -1.0;  // Negative power
        }

        // Apply deceleration zone
        if (Math.abs(error) < TURRET_DECEL_ZONE) {
            // Slow down as we approach target
            double slowdownFactor = (Math.abs(error) / TURRET_DECEL_ZONE)/2;
            power = power * Math.max(0.3, slowdownFactor);  // Don't go below 30% power
        }

        turret.setPower(power);
        telemetry.addData("Turret Status", "MOVING");
        telemetry.addData("Turret Power", "%.2f", power);
    }

    /**
     * SOLUTION 2: Advanced control with velocity-based prediction
     * Moves to ABSOLUTE target position (e.g., 15°) from any starting position
     */
    private void controlTurretAdvanced(double currentPosition, double targetPosition) {
        // Calculate velocity (degrees per second)
        double dt = turretTimer.seconds();
        if (dt > 0.001) {
            turretVelocity = (currentPosition - lastTurretAngle) / dt;
            lastTurretAngle = currentPosition;
            turretTimer.reset();
        }


        double error = targetPosition - currentPosition;  // Positive = move forward, Negative = move backward

        // Check if we've reached the target (position AND velocity must be low)
        if (Math.abs(error) < TURRET_TOLERANCE && Math.abs(turretVelocity) < 10) {
            turret.setPower(0);
            telemetry.addData("Turret Status", "AT TARGET");
            return;
        }



        // Predictive stopping based on current velocity
        // If moving fast, stop farther away
        double predictiveStopDistance = Math.max(TURRET_STOP_EARLY, Math.abs(turretVelocity) * 0.05);

        if (Math.abs(error) < predictiveStopDistance) {
            turret.setPower(0);
            telemetry.addData("Turret Status", "PREDICTIVE STOP");
            telemetry.addData("Stop Distance", "%.1f°", predictiveStopDistance);
            return;
        }

        // Determine direction and calculate power
        double power;

        if (error < 0) {
            // Need to move forward (increase angle)
            power = 1.0;
        } else {
            // Need to move backward (decrease angle)
            power = -1.0;
        }

        // Apply deceleration zone
        if (Math.abs(error) < TURRET_DECEL_ZONE) {
            double slowdownFactor = Math.abs(error) / TURRET_DECEL_ZONE;
            power = power * Math.max(0.25, slowdownFactor);
        }

        turret.setPower(power);
        telemetry.addData("Turret Status", "MOVING");
        telemetry.addData("Turret Power", "%.2f", power);
        telemetry.addData("Turret Velocity", "%.1f°/s", turretVelocity);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        turret = hardwareMap.get(CRServo.class, "turret");
        turretPOT = hardwareMap.get(AnalogInput.class, "turretPOT");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-152.4, 69.85, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blueStart.mirror());
        follower.update();

//        panelsTelemetry = PanelsTelemetry.getFtcTelemetry();

        follower.startTeleopDrive();

        turret.setPower(0);

        initEncoder(turretPOT.getVoltage());
        reset();

        // Initialize turret tracking
        lastTurretAngle = -(turnCount*52+(mapValue(turretPOT.getVoltage())*52));
        turretTimer.reset();

        waitForStart();

        while(opModeIsActive()){

            Pose2D pos = pinpoint.getPosition();

            double yaw = pos.getHeading(AngleUnit.DEGREES);

            double turretAngle = alignTurret(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    yaw,
                    redGoalPs);
            double rawDeg = (turretPOT.getVoltage()/3.3)*360;
            pinpoint.update();

            double largeGearCurrentPosition = -(turnCount*52+(mapValue(turretPOT.getVoltage())*52));

            // Display telemetry
            telemetry.addData("Yaw (degrees)", yaw);
            telemetry.addData("X TICKS", pinpoint.getEncoderX());
            telemetry.addData("Y TICKS", pinpoint.getEncoderY());
            telemetry.addData("Follower X", follower.getPose().getX());
            telemetry.addData("Follower Y", follower.getPose().getY());
            telemetry.addData("turretAngle", turretAngle);
            telemetry.addData("current turret ang", rawDeg);
            telemetry.addData("voltage", turretPOT.getVoltage());
            telemetry.addData("relative deg ", updateEncoder(turretPOT.getVoltage()));
            telemetry.addData("Large Gear Position", "%.1f°", largeGearCurrentPosition);
            telemetry.addData("Target Position", "%.1f°", TURRET_TARGET);
            telemetry.addData("Error", "%.1f°", TURRET_TARGET - largeGearCurrentPosition);

            // Your original code (commented out)
//            if(gamepad1.left_bumper){
//                shooter1.setVelocity(1000);
//                shooter2.setVelocity(1000);
//            } else {
//                shooter1.setVelocity(0);
//                shooter2.setVelocity(0);
//            }
//
//            if(gamepad1.right_trigger > 0.5){
//                intake.setPower(1);
//            } else {
//                intake.setPower(0);
//            }
//
//            if(gamepad1.right_bumper){
//                indexer.setPower(1);
//            } else {
//                indexer.setPower(0);
//            }


            telemetry.addData("red Shooter", redShooter);
            // NEW IMPROVED TURRET CONTROL
            if (gamepad1.right_bumper){
//                redShooter++;
//                turret.setPower(1);

                // CHOOSE ONE:
                // Option 1: Simple with deceleration (easier to tune)
//                controlTurretSimple(largeGearCurrentPosition, TURRET_TARGET);

                // Option 2: Advanced with velocity prediction (more accurate)
                // controlTurretAdvanced(largeGearCurrentPosition, TURRET_TARGET);
            }
            if (gamepad1.b) {
                shooter1.setPower(1);
                shooter2.setPower(1);
            }

            if (gamepad1.leftBumperWasReleased()){
                controlTurretSimple(largeGearCurrentPosition,turretAngle);
            }

//            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            telemetry.update();
            follower.update();
        }
    }

    public double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return  turretAngle + turretOffset;
    }
}