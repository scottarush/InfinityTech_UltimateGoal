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

package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is the class to be used for the teleop dual driver mode with theSpeedBot
 * Control layout is:
 * 1. Tank drive on both joysticks.  Gamepad 1 is primary but if both joysticks are idle,
 *    then Gamepad2 joysticks work otherwise ignored.
 * 2. left bumper on either controller toggles front hooks
 * 3. right bumper on either controller toggles hand between open and closed
 * 4. left trigger on either controller lowers crane
 * 5. right trigger on either controller raises crane.
 * 6. a button on either controller moves hand to retracted position
 */
@TeleOp(name="SpeedBotDualDriver", group="Robot")
@Disabled
public class SpeedBotDualDriverOpmode extends OpMode{
    private static final int MIN_BUTTON_UPDATE_TIME_MS = 200;

    private long mLastUpdateTime = 0L;

    private CraneSpeedBot robot  = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new CraneSpeedBot(this,false);
        /*
         * Initialize the robot.  Be sure to catch exception and dump out as
         * the exception string will have details of what didn't initialize and
         * we can figure out what went wrong.
         */
        String statusMsg = "Success";
        try{
            robot.init();
        }
        catch(Exception e){
            statusMsg = "InitErrors:"+e.getMessage();
        }
        telemetry.addData("Status",statusMsg);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // Allow gamepad 2 is idle (i.e. gamepad1 all less than min threshold.)
        double minThreshold = 0.01d;
        if ((Math.abs(xleft) < minThreshold) &&
                (Math.abs(yleft) < minThreshold) &&
                (Math.abs(xright) < minThreshold) &&
                (Math.abs(yright) < minThreshold)){
            xleft = gamepad2.left_stick_x;
            yleft = -gamepad2.left_stick_y;
            xright = gamepad2.right_stick_x;
            yright = -gamepad2.right_stick_y;
        }
        // Now apply nonlinear joystick gain to each raw value
        xleft = applyJoystickGain(xleft);
        yleft = applyJoystickGain(yleft);
        xright = applyJoystickGain(xright);
        yright = applyJoystickGain(yright);

        robot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

        // Limit update rate to the other controls
        long delta = System.currentTimeMillis() - mLastUpdateTime;
        if (delta > MIN_BUTTON_UPDATE_TIME_MS) {
            mLastUpdateTime = System.currentTimeMillis();

            // Use the right bumper to open and close the hand
            boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;
            if (rightBumper) {
                switch(robot.getCrane().getHandPosition()){
                    case Crane.HAND_CLOSED:
                        robot.getCrane().setHandPosition(Crane.HAND_OPEN);
                        break;
                    case Crane.HAND_OPEN:
                        robot.getCrane().setHandPosition(Crane.HAND_CLOSED);
                        break;
                    case Crane.HAND_RETRACTED:
                        robot.getCrane().setHandPosition(Crane.HAND_OPEN);
                        break;
                }
            }
            else {
                // Check for hand retract on button a
                boolean a = gamepad1.a || gamepad2.a;
                if (a) {
                    robot.getCrane().setHandPosition(Crane.HAND_RETRACTED);
                }
            }
            // Use left bumper to raise and lower the front hooks
            boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            if (leftBumper){
                if (robot.getFrontHooks().isOpen()){
                    robot.getFrontHooks().closeHooks();
                }
                else{
                    robot.getFrontHooks().openHooks();
                }
            }

            // Use the triggers to raise and lower the crane.  Allow either controller to
            // raise and lower, but if both triggers are pressed default to gamepad1 only
            double triggerThreshold = 0.05d;
            double leftTrigger = gamepad1.left_trigger;
            if (Math.abs(leftTrigger) < triggerThreshold) {
                leftTrigger = gamepad2.left_trigger;
            }
            double rightTrigger = gamepad1.right_trigger;
            if (Math.abs(rightTrigger) < triggerThreshold) {
                rightTrigger = gamepad2.right_trigger;
            }
            if (rightTrigger > triggerThreshold) {
                if (leftTrigger < triggerThreshold) {
                    robot.getCrane().raiseManual(Math.abs(rightTrigger));
                }
            } else if (leftTrigger > triggerThreshold) {
                if (rightTrigger < triggerThreshold) {
                    robot.getCrane().lowerManual(Math.abs(leftTrigger));
                }
            } else {
                // Neither trigger so stop the crane in case it was moving
                robot.getCrane().stop();
            }

//        // Process the autoramp Crane inputs on either controller's dpad Up or Down
//        boolean dpadUp = gamepad1.dpad_up || gamepad1.dpad_up;
//        boolean dpadDown = gamepad2.dpad_down || gamepad2.dpad_down;
//        if (dpadUp){
//            robot.getCrane().raiseManualRamp();
//        }
//        else if (dpadDown){
//            robot.getCrane().lowerManualRamp();
//        }
//        else {
//            robot.getCrane().stop();
//        }
        }
    }

    /**
     * helper function for control gain
     */
    private double applyJoystickGain(double input){
        double output = input * input;
        return output * Math.signum(input);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.getDrivetrain().stop();
    }


}
