/***
 * swerbe
 * @author David Grieas - 14212 MetroBotics - for 23403 - C{}de C<>nduct<>rs
 * coding for qualifier 2 - dec 6th
 * started coding at 12/2/25  @  7:15 pm
 * finished coding at 12/2/25  @  11:27 pm
***/
package org.firstinspires.ftc.teamcode.testCode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.SwerveSS;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

@Config
@Configurable
@Disabled
@TeleOp(name = "swerbe", group = "test_ftc14212")
public class swerbe extends LinearOpMode {
    /**
     * SWERBE BY DAVID
     * @author David Grieas - 14212 MetroBotics
    **/
    public boolean debugMode = true;
    @Override
    public void runOpMode() {
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        SwerveSS swerve = new SwerveSS(hardwareMap);
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "Swerve INIT DONE");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Gamepad gamepad1p = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
                double forward = -gamepad1p.left_stick_y;
                double strafe  =  gamepad1p.left_stick_x;
                double rotate  =  gamepad1p.right_stick_x;
                // X-LOCK TOGGLE
                if (gamepad1p.b) {
                    swerve.setXLock();
                }
                // if (gamepad1p.a) swerve.autoAlign(target, heading);
                // regular driving (auto exits x-lock if input detected)
                swerve.drive(forward, strafe, rotate);
                telemetryM.addLine(swerve.telemetry());
                telemetryM.update();
            }
        }
    }
}
