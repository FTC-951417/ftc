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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCardbot;



@Autonomous(name="Auto Blue Right", group="Blue")
public class CardbotAutoBlueRight extends AutoBase {




    @Override
    public void runOpMode() {
        initOpMode(); // Start IMU and map hardware
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.leftClaw.setPosition(robot.LEFT_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLOSED); // Close claws

        robot.sensorArm.setPosition(1);
        sleep(1500); // Wait for arm to move!
        //robot.sensorArm.setPosition(0);

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
        int i = -1;
        runtime.reset();
        while (!(end) && opModeIsActive() && runtime.seconds() < 4.0) {
            i++;
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

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

        if(colors.red > colors.blue) {
            robot.sensorArm.setPosition(1);
            { // BACKWARD
                //robot.reverseAll();
                encoderDrive(0.1, -3, 5.0);
                //robot.reverseAll();
            }
            robot.sensorArm.setPosition(0);
            dirId = 1;
        }
        if(colors.blue > colors.red) {
            robot.sensorArm.setPosition(1);
            { // FORWARD

                encoderDrive(0.1, 3  , 5.0);

            }
            robot.sensorArm.setPosition(0);
            dirId = 2;
        }
        turnToDegree(0.2, 0);
        if(dirId == 2) { // BACKWARD

            //robot.reverseAll();
            encoderDrive(0.2, -4, 5.0); // Account for forward slide
            //robot.reverseAll();

        } else if(dirId == 1) { // FORWARD

            encoderDrive(0.1, 3, 5.0);

        } else {
            requestOpModeStop(); // Error
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

        turnToDegree(0.3, 0);

        robot.mainArm.setPower(-0.8);  // Move arm up so it doesn't create friction

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {}



        robot.mainArm.setPower(0);  // Stop moving arm after 300ms

        //encoderDrive(0.3, -10, 4.0); // Go backward 10in

        turnToDegree(0.3, 0);

        encoderDrive(0.5, -24, -30, 5); // Turn right and outward

        //encoderDrive(0.3, 3, 5);

        turnToDegree(0.3, -90);

        if(dirId == 1) {
            encoderDrive(0.5, -4, -6);
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {}


        if(vuMarkAnswer == RelicRecoveryVuMark.CENTER) {

            //Turn Right 10 inches

            turnToDegree(0.3, -90);

            encoderDrive(0.5,15, 5.0);

        }
        if(vuMarkAnswer == RelicRecoveryVuMark.RIGHT) {
            //Turn Right 7-8 inches


            encoderDrive(0.3, -4, 2.0);
            turnToDegree(0.3, dirId == 1 ? -102 : -102);

            encoderDrive(0.5, 21,5.0);

            //encoderDrive(0.3, -4, 5.0);
            //runtime.reset();
            //while(opModeIsActive() && runtime.seconds() < 0.3);
            //encoderDrive(0.3, 4, 5.0);
        }
        if(vuMarkAnswer == RelicRecoveryVuMark.LEFT) {
            encoderDrive(0.3, -4, 2);

            turnToDegree(0.3, -77);

            encoderDrive(0.5,17,5.0);
        }
        robot.leftClaw.setPosition(robot.LEFT_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_OPEN);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {}

        encoderDrive(0.5, -4, 5);




        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




}


