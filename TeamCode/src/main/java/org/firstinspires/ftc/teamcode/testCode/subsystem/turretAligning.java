package org.firstinspires.ftc.teamcode.testCode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.testCode.PID.turret.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
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
    public static double ledCpos = 1;
    public static double turretTpos = 0;
    boolean redSide;
    TurretSS turretSS;
    TelemetryM telemetryM;
    private static CachingServo led; // 2x gobilda led lights RGB
    Follower follower;
    @Override
    public void init() {
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", MainV1E.Alliance.RED, MainV1E.Alliance.BLUE))
                .prompt("start_pos", new OptionPrompt<>("Starting Position", MainV1E.StartPos.FAR, MainV1E.StartPos.CLOSE))
                .onComplete(this::onPromptsComplete);
        telemetryM = new TelemetryM(telemetry, debugMode);
        follower = Constants.createFollower(hardwareMap);
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        DcMotorEx encoder = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        turretSS = new TurretSS(turretPID, PIDTuneTurret.F, turret, encoder, PIDTuneTurret.TPR, PIDTuneTurret.ratio, turretOffset);
        led.setPosition(ledCpos);
        encoder.setDirection(DcMotorEx.Direction.REVERSE);
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
        Gamepad gamepad1p = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
        follower.setTeleOpDrive(-gamepad1p.left_stick_y, -gamepad1p.left_stick_x, -gamepad1p.right_stick_x, true);
        if (turretSS.alignTurret() == 0 && turretTpos != 0) turretSS.updateTurretTpos(turretTpos);
        else if (turretSS.alignTurret() != 0) turretTpos = turretSS.turretTpos;
        turretSS.update(follower);
        if (gamepad1p.left_bumper) {
            ledCpos = 0.388;
            turretSS.alignTurret();
        } else {
            if (redSide) ledCpos = 0.278;
            else ledCpos = 0.611;
        }
        telemetryM.addLine(turretSS.telemetry());
        telemetryM.update();
    }
}
