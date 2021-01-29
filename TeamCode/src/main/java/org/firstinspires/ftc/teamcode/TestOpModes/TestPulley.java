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

package org.firstinspires.ftc.teamcode.TestOpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

import java.util.SortedMap;

/*
 * This is an example LinearOpMode that shows how to use
 * a REV Robotics Touch Sensor.
 *
 * It assumes that the touch sensor is configured with a name of "sensor_digital".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@TeleOp(name = "TestPulley", group = "Robot")
//@Disabled
public class TestPulley extends LinearOpMode {
    private Shooter mShooter = null;

    private boolean mLastToggleButtonState = false;


    @Override
    public void runOpMode() {
        mShooter = new Shooter(this);
        try {
            mShooter.init(hardwareMap);
        }
        catch (Exception e){
            telemetry.addLine(e.getMessage());
        }

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            boolean edgeDetect = false;
            boolean buttonState = gamepad1.y;
            if (buttonState) {
                // button pressed. check if last false
                if (mLastToggleButtonState == false) {
                    edgeDetect = true;
                }
            }
            mLastToggleButtonState = buttonState;

            if (edgeDetect){
                // Toggle the pulley position
                switch(mShooter.getLoaderPulleyPosition()){
                    case Shooter.LOADER_PULLEY_POSITION_HIGH:
                        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                        break;
                    case Shooter.LOADER_PULLEY_POSITION_LOW:
                        mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_HIGH);
                        break;
                    case Shooter.LOADER_PULLEY_POSITION_MIDDLE:
                        if (mShooter.isLoaderPulleyMoving()) {
                            mShooter.stopLoaderPulley();
                        }
                        else{
                            mShooter.setLoaderPulleyPosition(Shooter.LOADER_PULLEY_POSITION_LOW);
                        }
                        break;
                }

            }

        }
    }
}
