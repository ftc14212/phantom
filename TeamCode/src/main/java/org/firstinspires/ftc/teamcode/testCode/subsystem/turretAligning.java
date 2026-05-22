package org.firstinspires.ftc.teamcode.testCode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDDualTuneTurret;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.PPPoint;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.vars.MainV1E;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "turretSS", group = "test_ftc14212")
public class turretAligning extends OpMode {
    private final Prompter prompter = new Prompter(this);
    public boolean debugMode = true;
    public static double turretOffset = 0;
    boolean redSide;
    TurretSS turretSS;
    TelemetryM telemetryM;
    CachingServo led; // 2x gobilda led lights RGB
    Follower follower;
    public static PPPoint.pose bluePos = new PPPoint.pose(6, 138);
    public static PPPoint.pose redPos = new PPPoint.pose(6, 138);
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .onComplete(this::onPromptsComplete);
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        DcMotorEx encoder = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        encoder.setDirection(DcMotorEx.Direction.REVERSE);
        led.setPosition(1);
        turretSS = new TurretSS(turret, encoder, PIDTuneTurret.pidf, MainV1E.lastTurretPos);
        turretSS.setWrapAngles(-170, 170);
        turretSS.update(follower);
        telemetryM.update();
    }

    public void onPromptsComplete() {
        MainV1E.Alliance alliance = prompter.get("alliance");
        MainV1E.StartPos startPos = prompter.get("start_pos");
        if (startPos == MainV1E.StartPos.FAR) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(87.5, 8.3, Math.toRadians(90)));
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(new Pose(56.5, 8.3, Math.toRadians(90)));
        }
        if (startPos == MainV1E.StartPos.CLOSE) {
            if (alliance == MainV1E.Alliance.RED) follower.setStartingPose(new Pose(126, 8.3, Math.PI - Math.toRadians(144)));
            if (alliance == MainV1E.Alliance.BLUE) follower.setStartingPose(new Pose(18, 119, Math.toRadians(144)));
        }
        redSide = alliance == MainV1E.Alliance.RED;
        turretSS.setRedSide(redSide);
        if (redSide) led.setPosition(0.278);
        else led.setPosition(0.611);
        telemetryM.addLine("PHANTOM Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.addData(true, "Alliance", alliance);
        telemetryM.addData(true, "Starting pos", startPos);
        telemetryM.update();
    }

    @Override
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // poses
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        turretSS.setOffset(turretOffset);
        turretSS.setPoses(bluePos.getPose(), redPos.getPose());
        turretSS.update(follower);
        if (gamepad1.left_bumper) {
            turretSS.align();
            led.setPosition(0.388);
        } else {
            led.setPosition(redSide ? 0.278 : 0.611);
            turretSS.reset();
        }
        telemetryM.addLine(turretSS.telemetry());
        telemetryM.update();
    }
}
