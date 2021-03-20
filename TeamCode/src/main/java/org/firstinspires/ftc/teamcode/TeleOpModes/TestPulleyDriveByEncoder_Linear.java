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

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="Test Pulley Drive By Encoder", group="Robot")
@Disabled
public class TestPulleyDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         mPulley;
    private RevTouchSensor  mPulleyHighTouchSensor;
    private ElapsedTime     runtime = new ElapsedTime();
    private int             pulleyEncoderValueLow = 0;
    private int             pulleyEncoderValueHigh =50;
    private int             pulleyEncoderValueCurrent;

    private enum PULLEY_STATES {
        LOW, MID, HIGH, UNKNOWN
    }

    private PULLEY_STATES pulleyState;

    private double power = 0.15;

    @Override
    public void runOpMode() {
        /*
            Initialize Hardware
         */

        mPulley = hardwareMap.get(DcMotor.class, "loader");
        mPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mPulleyHighTouchSensor = hardwareMap.get(RevTouchSensor.class, "highlsw");

        // Pulley will start in LOW state
        // Reset Motor Encoder
        mPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyEncoderValueCurrent = mPulley.getCurrentPosition();
        pulleyEncoderValueLow = pulleyEncoderValueCurrent;
        pulleyState = PULLEY_STATES.LOW;

        // Set Motor Mode to RUN_USING_ENCODER
        mPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message
        telemetry.addData("Low Encoder Value", "%7d", pulleyEncoderValueLow);
        telemetry.addData("High Encoder Value", "%7d", pulleyEncoderValueHigh);
        telemetry.addData("Current Encoder Value", "%7d", pulleyEncoderValueCurrent);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            mPulley.setDirection(DcMotorSimple.Direction.REVERSE);

            while (!mPulleyHighTouchSensor.isPressed()) {
                mPulley.setPower(power);
                pulleyEncoderValueCurrent = mPulley.getCurrentPosition();
                // Send telemetry message
                telemetry.addData("Low Encoder Value", "%7d", pulleyEncoderValueLow);
                telemetry.addData("High Encoder Value", "%7d", pulleyEncoderValueHigh);
                telemetry.addData("Current Encoder Value", "%7d", pulleyEncoderValueCurrent);
                telemetry.update();
            }

            // When Pressed, stop the Pulley Motor and set the High Position to current position
            mPulley.setPower(0.0);
            pulleyEncoderValueCurrent = mPulley.getCurrentPosition();
            pulleyEncoderValueHigh = pulleyEncoderValueCurrent;

            // Reverse the direction and run to position
            mPulley.setDirection(DcMotorSimple.Direction.FORWARD);
            mPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mPulley.setTargetPosition(pulleyEncoderValueLow);
            mPulley.setPower(power);

            while (mPulley.isBusy()) {
                pulleyEncoderValueCurrent = mPulley.getCurrentPosition();
                // Send telemetry message
                telemetry.addData("Low Encoder Value", "%7d", pulleyEncoderValueLow);
                telemetry.addData("High Encoder Value", "%7d", pulleyEncoderValueHigh);
                telemetry.addData("Current Encoder Value", "%7d", pulleyEncoderValueCurrent);
                telemetry.update();
            }

            pulleyState = PULLEY_STATES.LOW;
            mPulley.setPower(0.0);
        }
    }
}
