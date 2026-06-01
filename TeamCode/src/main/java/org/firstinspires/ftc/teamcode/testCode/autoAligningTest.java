package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@Configurable
@TeleOp(name = "autoAligningTest", group = "test_ftc14212")
public class autoAligningTest extends LinearOpMode {
    boolean debugMode = true;
    @Override
    public void runOpMode() {
        // hardware
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(2);
        limelight.start();
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");//hi
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                telemetryM.addData("tx", result.getTx());
                telemetryM.addData("ty", result.getTy());
                telemetryM.addData("ta", result.getTa());

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                    double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                    double distance = fiducial.getRobotPoseTargetSpace().getPosition().y;
                    Pose3D relative = fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
                    Pose3D camPose = fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
                    Pose3D robotPose = fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
                    telemetryM.addData("Fiducial " + id, "is " + distance + " meters away");
                    telemetryM.addData("Fiducial " + id, "coords are (" + x + "," + y + ")");
                    telemetryM.addData("Fiducial " + id, "relative: " + relative.toString());
                    telemetryM.addData("Fiducial " + id, "camPose: " + camPose.toString());
                    telemetryM.addData("Fiducial " + id, "robotPose: " + robotPose.toString());
                }

                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
            limelight.stop();
        }
    }
}
