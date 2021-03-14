/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevTouchSensor;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test Pulley Touch Sensors", group = "Robot")
//@Disabled
public class TestPulleyTouchSensors extends LinearOpMode {

    RevTouchSensor mPulleyHighTouchSensor = null;
    RevTouchSensor mPulleyLowTouchSensor = null;

    enum PulleyStates {
        LOW, HIGH, MIDDLE, ERROR
    }

    @Override
    public void runOpMode() {


        mPulleyHighTouchSensor = hardwareMap.get(RevTouchSensor.class, "highlsw");
        // mPulleyLowTouchSensor = hardwareMap.get(RevTouchSensor.class,"lowlsw");

        // Wait for the start button
        telemetry.addData(">", "Press Start to test Touch Sensors" );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            boolean isHighTouchSensorPressed = mPulleyHighTouchSensor.isPressed();
            boolean isLowTouchSensorPressed = mPulleyLowTouchSensor.isPressed();
            PulleyStates pulleyState;
            String pulleyStateMessage;

            if (isHighTouchSensorPressed && isLowTouchSensorPressed) {
                pulleyState = PulleyStates.ERROR;
            } else if (isHighTouchSensorPressed) {
                pulleyState = PulleyStates.HIGH;
            } else if (isLowTouchSensorPressed) {
                pulleyState = PulleyStates.LOW;
            } else {
                pulleyState = PulleyStates.MIDDLE;
            }

            switch (pulleyState){
                case LOW:
                    pulleyStateMessage = "LOW";
                    break;
                case HIGH:
                    pulleyStateMessage = "HIGH";
                    break;
                case MIDDLE:
                    pulleyStateMessage = "MIDDLE";
                    break;
                case ERROR:
                    pulleyStateMessage = "ERROR";
                    break;
                default:
                    pulleyStateMessage = "*DEFAULT*";
                    break;
            }

            // Display the current value
            telemetry.addData("Pulley State ", pulleyStateMessage);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();


        }


    }
}
