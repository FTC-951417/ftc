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

package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCardbot;



@Autonomous(name="Auto Blue Right", group="Blue")
public class CardbotAutoBlueRight extends AutoBase {




    @Override
    public void runOpMode() {
        alliance = new Alliance("blue");
        initOpMode();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        startIMU();


        robot.leftClaw.setPosition (robot.LEFT_CLOSED);  //CLOSE CLAWS
        robot.rightClaw.setPosition (robot.RIGHT_CLOSED);





        robot.sensorArm.setPosition(1);   // Put down arm for color reading
        sleep(1500); // Wait for arm to move!
        //robot.sensorArm.setPosition(0.35);

        int dirId = 0;
        while(colors == null && opModeIsActive()){
            try {
                getColor();
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }

        while(colors.red <= 0.005 || colors.blue <= 0.005 && opModeIsActive()) {
            try {
                getColor();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        sleep(500);

        if(colors.red > colors.blue) {
            robot.sensorArm.setPosition(1);
            { // Backward
                robot.reverseAll();
                encoderDrive(0.2, 4, 4, 5.0);
                robot.reverseAll();

            }
            robot.sensorArm.setPosition(0.35);
            dirId = 1;
        }
        if(colors.blue > colors.red) {
            robot.sensorArm.setPosition(1);
            { // Forward

                encoderDrive(0.2, 4, 4, 5.0);

            }
            robot.sensorArm.setPosition(0.35);
            dirId = 2;
        }

        if(dirId == 2) { // Forward
            robot.reverseAll();
            encoderDrive(0.2, 4, 4, 5.0);
            robot.reverseAll();
        } else if(dirId == 1) { // Forward

            encoderDrive(0.2, 4, 4, 5.0);

        } else {
            requestOpModeStop(); //Error
        }

        robot.mainArm.setPower(-0.8);  // Move arm up so it doesn't create friction
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

        robot.mainArm.setPower(0);  // Stop moving arm after 800ms

        //encoderDrive(0.3, 5, 5);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         */
        parameters.vuforiaLicenseKey = "AbN5bVn/////AAAAGYB9Mi8Lq0aOlHLXV+cCXXCCaDrZfFL4Mc9vNmpdEhix11+l5w6Kx7KEp5NIY43aTY1m0c454n9rAqxX5i5Kn0xHj0qq6jFjABBYaWQR+S0eagZoICVhfmHEhrFb4udp84Yqaj6Lgrkj5AwqO7pd3rfqOe39vAo1XY4w5KAADo0anBPFGPElHNlnhQ5HQOrbULoeMgOd+mm1SWHGsI+FafcuEj/hGn+AhxueQQ97+/+nEEmEsdNJzHSK9vQ0M6QeyhKR4imAN9AG87e3HgHYD1bM4E340H5T5Tio7BeSO9jhPat++RRbD1TqM5H994zSGyl7FpO3Gvl8s+SY5DXhML06IYnCKPvMNxK+/VLE3Yvy";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary;

        relicTrackables.activate();
        boolean end = false;
        boolean increment = true;

        RelicRecoveryVuMark vuMarkAnswer = null;
        while (!(end) && opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vuMarkAnswer = vuMark;
                telemetry.addData("VuMark", "%s visible", vuMark);
                end = true;
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        telemetry.addData("VuMark:", vuMarkAnswer);
        if(vuMarkAnswer == null || vuMarkAnswer == RelicRecoveryVuMark.UNKNOWN) {
            // Some error happened. Go right due to highest reliability
            vuMarkAnswer = RelicRecoveryVuMark.RIGHT;
        }
        robot.reverseAll();
        encoderDrive(0.4, 20, 5.0); // Move back (this is because the VuMark is on the opposite side of the scoring area)
        robot.reverseAll();
        // Turn around (left/counterclockwise), this is because the right side has the color sensor and the robot is now "backwards"
        HardwareCardbot.reverse(robot.leftDrive);
        HardwareCardbot.reverse(robot.leftDrive2);
        encoderDrive(1,25,5.0);
        HardwareCardbot.reverse(robot.leftDrive);
        HardwareCardbot.reverse(robot.leftDrive2);

        vuMarkAnswer = RelicRecoveryVuMark.CENTER;

        if(vuMarkAnswer == RelicRecoveryVuMark.CENTER) {
            //Turn Left 15 inches
            robot.reverseAll();
            encoderDrive(0.2, 3, 5.0);
            robot.reverseAll();
            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);
            encoderDrive(0.5, 12, 5.0);
            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);
            encoderDrive(0.5,9, 5.0);
        }
        if(vuMarkAnswer == RelicRecoveryVuMark.LEFT) {

            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            encoderDrive(0.3, 18, 7.0);
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

            encoderDrive(0.5,4, 5.0);
        }
        if(vuMarkAnswer == RelicRecoveryVuMark.RIGHT) {
            //Turn Left 12 inches

            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);
            encoderDrive(1, 5, 5.0);
            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);

            encoderDrive(0.5, 3, 5.0); // Go forward 22in
        }
        robot.leftClaw.setPosition(robot.LEFT_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_OPEN);  // Open claw to free glyph
        //encoderDrive(0.5, -3, 5.0); Keeps glyph in place better



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




}


