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

package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Manual Mode", group="Cardbot")
public class CardbotTeleopTank_Iterative extends OpMode {

    /* Declare OpMode members. */
    HardwareCardbot robot       = new HardwareCardbot();

    private double lgrip = 0;
    private double rgrip = 1;





    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        /* FTC Relic Recovery rules state
         *
         */
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello drivers!");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say2", "Initiating Manual Drive Mode...");
        telemetry.update();
        try {
            Thread.sleep(100);
        } catch(InterruptedException e){ telemetry.addData("Say3", "Sleep interrupted! Tell a programmer!"); }
        telemetry.addData("Say4", "Hit play to start control.");
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
        telemetry.addData("Say", "Controllers, please designate control numbers.");
        telemetry.addData("Say2", "#1 hit Start+A");
        telemetry.addData("Say3", "#2 hit Start+B");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        robot.sensorArm.setPosition(0);



        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        /*
        // Compute speed of left,right motors.
        double deltaTime = time - prevTime;
        double leftSpeed = (robot.leftDrive.getCurrentPosition() - prevLeftEncoderPosition) /
                deltaTime;
        double rightSpeed = (robot.rightDrive.getCurrentPosition() - prevRightEncoderPosition) /
                deltaTime;
        double leftSpeed2 = (robot.leftDrive2.getCurrentPosition() - prevLeftEncoderPosition) /
                deltaTime;
        double rightSpeed2 = (robot.rightDrive2.getCurrentPosition() - prevRightEncoderPosition) /
                deltaTime;
        // Track last loop() values.
        prevTime = time;
        prevLeftEncoderPosition = robot.leftDrive.getCurrentPosition();
        prevRightEncoderPosition = robot.rightDrive.getCurrentPosition();
        prevLeft2EncoderPosition = robot.leftDrive2.getCurrentPosition();
        prevRight2EncoderPosition = robot.rightDrive2.getCurrentPosition();
        */

        if(gamepad1.right_trigger <= 0.2) { // Right trigger not pressed, full speed
            left = Range.clip(left, -1, 1);
            right = Range.clip(right, -1, 1);
        } else {  // ~1/3 speed, right trigger is pressed
            left = Range.clip(left, -0.32, 0.32);
            right = Range.clip(right, -0.32, 0.32);
        }
        setLeft(left);
        setRight(right);

        // Control servos with bumpers (Gamepad 2)

        if (gamepad2.left_trigger > 0.1) {
            // Reduce grip
            lgrip -= 0.03;
            rgrip += 0.03; // Right grip is reversed, 1 on right is 0 on left, etc.
        }
        if (gamepad2.right_trigger > 0.1) {
            // Increase grip
            lgrip += 0.03;
            rgrip -= 0.03; // Right grip is reversed, 1 on right is 0 on left, etc.
        }
        double distanceFromMid = 0.05;
        rgrip = Range.clip(rgrip, 0.4, 1); // Stop arm from crushing itself
        lgrip = Range.clip(lgrip, 0, 0.6); // * ^   ^    ^     ^       ^
        robot.leftClaw.setPosition(lgrip);
        robot.rightClaw.setPosition(rgrip);
        robot.leftClaw2.setPosition(lgrip);
        robot.rightClaw2.setPosition(rgrip);

        // Arm Functions

        double left2 = gamepad2.left_stick_y;
        double right2 = gamepad2.right_stick_y;
        robot.flipArm.setPower(left2);
        robot.mainArm.setPower(right2);

        // Free Motors / Lock Motors
        /*
        if (gamepad2.b && robot.flipArm.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            robot.flipArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (gamepad2.b && robot.flipArm.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT) {
            robot.flipArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        */

        //boolean isBraked = robot.flipArm.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE;

        // Telemetry
        telemetry.addData("Left Power", left);
        telemetry.addData("Right Power", right);
        telemetry.addData("Left Grip", lgrip);
        telemetry.addData("Right Grip", rgrip);
        //telemetry.addData("Arm Mode", isBraked ? "Brake" : "Free (WARNING! PLEASE RESET TO BRAKE WITH B on G2)");
        telemetry.update();
    }



    private void setLeft(double power){
        robot.leftDrive.setPower(power);
        robot.leftDrive2.setPower(power);
    }

    private void setRight(double power){
        robot.rightDrive.setPower(power);
        robot.rightDrive2.setPower(power);

    }


    /* Range functions */
    @SuppressWarnings("unused")
    public static boolean isNotInRangeExcludes(double in, double min, double max) {
        return in < min && in > max;
    }
    @SuppressWarnings("unused")
    public static boolean isNotInRangeIncludes(double in, double min, double max) {
        return in <= min && in >= max;
    }
    @SuppressWarnings("unused")
    public static boolean isInRangeExcludes(double in, double min, double max) {
        return in > min && in < max;
    }
    @SuppressWarnings("unused")
    public static boolean isInRangeIncludes(double in, double min, double max) {
        return in >= min && in <= max;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Robot is stopped!");
        telemetry.update();
    }


}
