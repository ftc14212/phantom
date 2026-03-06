package org.firstinspires.ftc.teamcode.testCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "leds", group = "test_ftc14212")
public class leds extends LinearOpMode {
    public static double ledCpos = 0.611;
    public static double stripsCpos = 0.7;
    @Override
    public void runOpMode() {
        // init
        Servo led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        Servo strips = new CachingServo(hardwareMap.get(Servo.class, "strips")); // 4x gobilda strip RGB lights
        led.setPosition(ledCpos);
        strips.setPosition(stripsCpos);
        waitForStart();
        if (opModeIsActive()) {
            // start teleop
            while (opModeIsActive()) {
                // teleop loop
                led.setPosition(ledCpos);
                strips.setPosition(stripsCpos);
            }
        }
    }
}
