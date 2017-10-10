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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        telemetry.addData("Say", "Initiating Manual Drive Mode...");
        telemetry.update();
        try {
            Thread.sleep(100);
        } catch(InterruptedException e){ telemetry.addData("Say", "Sleep interrupted! Tell a programmer!"); }
        telemetry.addData("Say", "Hit play to start control.");
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
        telemetry.addData("Say", "#1 hit Start+A");
        telemetry.addData("Say", "#2 hit Start+B");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {



        double left;
        double leftX;
        double rt;
        //double right;

        // Arcade mode
        left = -gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        double right = -gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;
        if(!(rightX > 0 | right > 0)) {
            double leftPower = left + leftX;
            double rightPower = left - leftX;
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);
            //right = -gamepad1.right_stick_y;
            setLeft(leftPower);
            setRight(rightPower);
        } else {
            if((rightX > 0.5 | rightX < -0.5) && !(right > 0)) {
                if(rightX > 0.5) {
                    robot.ld
                }
            } else if((rightX > 0.5 | rightX < -0.5)) {

            }
        }


        if(gamepad1.left_bumper) {
            // Reduce grip
            lgrip -= 0.02;
            rgrip += 0.02; // Right grip is reversed, 1 on right is 0 on left, etc.
        }
        if(gamepad1.right_bumper){
            // Increase grip
            lgrip += 0.02;
            rgrip -= 0.02; // Right grip is reversed, 1 on right is 0 on left, etc.
        }
        lgrip = Range.clip(lgrip, 0, 0.32); // Stop arm from crushing itself
        rgrip = Range.clip(rgrip, 0.58, 1); // * ^   ^    ^     ^       ^
        telemetry.addData("Grippage", "%" + String.valueOf(lgrip * 312.5));
        robot.leftClaw.setPosition(lgrip);
        robot.rightClaw.setPosition(rgrip);

        // Send telemetry message to signify robot running;
        telemetry.addData("Left Power", "%" + String.valueOf(leftPower * 100));
        telemetry.addData("Right Power", "%" + String.valueOf(rightPower * 100));
        telemetry.update();
    }

    public void setLeft(double power){
        robot.leftDrive.setPower(power);
        robot.leftDrive2.setPower(power);
    }

    public void setRight(double power){
        robot.rightDrive.setPower(power);
        robot.rightDrive2.setPower(power);
    }

    /* Start mekanum functions */
    /* rd  + ld2 = Diag left
       ld + rd2  = Diag right
       rd + rd2  = Strafe left
       ld + ld2  = Strafe right */

    public void diagLeft(boolean positive) {
        int pwr = positive ? 1 : -1;
        robot.rightDrive.setPower(pwr);
        robot.leftDrive2.setPower(pwr);
        // Opposites
        robot.rightDrive2.setPower(-pwr);
        robot.leftDrive.setPower(-pwr);
    }

    public void diagRight(boolean positive) {
        int pwr = positive ? 1 : -1;
        robot.leftDrive.setPower(pwr);
        robot.rightDrive2.setPower(pwr);
        // Opposites
        robot.rightDrive.setPower(-pwr);
        robot.leftDrive2.setPower(-pwr);
    }

    public void strafe(boolean goRight) {
        
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
