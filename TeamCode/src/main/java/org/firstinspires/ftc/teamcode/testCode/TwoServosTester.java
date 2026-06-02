package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Configurable
@TeleOp(name="Two Servo Tester", group = "test_ftc14212")
public class TwoServosTester extends LinearOpMode {
    public static double pos = 1;
    @Override
    public void runOpMode() {
        Servo servo1 = hardwareMap.get(Servo.class, "hoodR");
        Servo servo2 = hardwareMap.get(Servo.class, "hoodL");
        // hood: 0 - 0.37
        servo1.scaleRange(0, 0.37);
        servo2.scaleRange(0, 0.37);
        servo1.setPosition(0);
        servo2.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                servo1.setPosition(pos);
                servo2.setPosition(pos);
            }
        }
    }
}