package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Configurable
@TeleOp(name="Servo Tester", group = "test_ftc14212")
public class ServoTester extends LinearOpMode {
    public static double pos = 1;
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "pivot");
        servo.scaleRange(0, 0.375); // stopper: 0.42 - 0.55  |  pivot: 0 - 0.1 - 0.15 - 0.375
        servo.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                servo.setPosition(pos);
            }
        }
    }
}