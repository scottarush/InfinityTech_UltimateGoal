package org.firstinspires.ftc.teamcode.TestOpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp
@Disabled
public class TestRevTouch extends LinearOpMode {

        // Define variables for our touch sensor and motor
        TouchSensor touch;
        DcMotor motor;

        @Override
        public void runOpMode() {
            // Get the touch sensor and motor from hardwareMap
            touch = hardwareMap.get(TouchSensor.class, "grabberlsw");
            motor = hardwareMap.get(DcMotor.class, "rr");

            // Wait for the play button to be pressed
            waitForStart();

            // Loop while the Op Mode is running
            while (opModeIsActive()) {
                // If the touch sensor is pressed, stop the motor
                if (touch.isPressed()) {
                    motor.setPower(0);
                } else { // Otherwise, run the motor
                    motor.setPower(0.3);
                }
            }
        }
}
