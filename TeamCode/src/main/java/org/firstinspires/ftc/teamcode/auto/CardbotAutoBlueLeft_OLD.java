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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCardbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Blue !!!OLD!!! Left", group="Left")
public class CardbotAutoBlueLeft_OLD extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCardbot         robot   = new HardwareCardbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     COUNTS_PER_MOTOR_REV_ARM    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION_ARM    = 8.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES_ARM   = 1.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_ARM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public NormalizedRGBA colors;
    public Alliance alliance;


    public static final String TAG = "Vuforia VuMark Cardbot";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        alliance = new Alliance("blue");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        double LEFT_OPEN = 0.45;
        double LEFT_CLOSED = 1;
        double RIGHT_OPEN = 0.45;
        double RIGHT_CLOSED = 0;
        robot.leftClaw.setPosition(robot.LEFT_CLOSED);
        robot.rightClaw.setPosition(robot.RIGHT_CLOSED);








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
        while (!(end) && opModeIsActive()) {
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

        sleep(500);

        if(colors.red > colors.blue) {
            robot.sensorArm.setPosition(1);
            { // Turn Right (BACKWARD)
                robot.reverseAll();
                encoderDrive(0.2, 4, 4, 5.0);
                robot.reverseAll();
            }
            robot.sensorArm.setPosition(0);
            dirId = 1;
        }
        if(colors.blue > colors.red) {
            robot.sensorArm.setPosition(1);
            { // Turn Left (FORWARD)

                encoderDrive(0.2, 4, 4, 5.0);

            }
            robot.sensorArm.setPosition(0);
            dirId = 2;
        }

        if(dirId == 2) { // Turn Right (BACKWARD)

            robot.reverseAll();
            encoderDrive(0.2, 4, 4, 5.0);
            robot.reverseAll();

        } else if(dirId == 1) { // Turn Left (FORWARD)

            encoderDrive(0.2, 4, 4, 5.0);

        } else {
            requestOpModeStop(); // Error
        }

        sleep(500);

        robot.mainArm.setPower(-0.8);  // Move arm up so it doesn't create friction
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

        robot.mainArm.setPower(0);  // Stop moving arm after 800ms

        robot.mainArm.setPower(0);

        HardwareCardbot.reverse(robot.leftDrive);
        HardwareCardbot.reverse(robot.leftDrive2);
        encoderDrive(0.3, 28, 5);
        HardwareCardbot.reverse(robot.leftDrive);
        HardwareCardbot.reverse(robot.leftDrive2);

        encoderDrive(0.3, 3, 5);

        sleep(1000); // Wait for motors to come to rest


        if(vuMarkAnswer == RelicRecoveryVuMark.CENTER) {
            //Turn Right 7-8 inches

            HardwareCardbot.reverse(robot.rightDrive);
            HardwareCardbot.reverse(robot.rightDrive2);
            encoderDrive(0.3, 3, 5.0);
            HardwareCardbot.reverse(robot.rightDrive);
            HardwareCardbot.reverse(robot.rightDrive2);

            encoderDrive(0.5,24,22,5.0);
        }
        if(vuMarkAnswer == RelicRecoveryVuMark.RIGHT) {
            //Turn Right 10 inches

            HardwareCardbot.reverse(robot.rightDrive);
            HardwareCardbot.reverse(robot.rightDrive2);
            encoderDrive(0.3, 3, 5.0);
            HardwareCardbot.reverse(robot.rightDrive);
            HardwareCardbot.reverse(robot.rightDrive2);

            encoderDrive(0.5,24, 5.0);
        }
        if(vuMarkAnswer == RelicRecoveryVuMark.LEFT) {


            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);
            encoderDrive(0.3, 1, 5.0);
            HardwareCardbot.reverse(robot.leftDrive);
            HardwareCardbot.reverse(robot.leftDrive2);

            encoderDrive(0.5,24,22,5.0);
        }
        robot.leftClaw.setPosition(robot.LEFT_OPEN);
        robot.rightClaw.setPosition(robot.RIGHT_OPEN);
        encoderDrive(0.3, -3, 5);




        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public double diameter = 16;
    public double circumference = diameter * Math.PI;

    public double degreesToInches(int degrees) {
        return (degrees / 360) * circumference;
    }

    public double inchesToDegrees(double inches) {
        return 360 * (inches / circumference);
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, inches, timeoutS);
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDrive2.setTargetPosition(newLeftTarget2);
            robot.rightDrive2.setTargetPosition(newRightTarget2);


            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.leftDrive2.setPower(Math.abs(speed));
            robot.rightDrive2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftDrive2.isBusy() && robot.rightDrive2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderArm(double speed,
                           double inches,
                           double timeoutS) {
        int newTarget;
        robot.mainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.leftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_ARM);
            robot.mainArm.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            robot.mainArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.mainArm.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftDrive2.isBusy() && robot.rightDrive2.isBusy())) {

                telemetry.addData("Arm", "Moving");
                telemetry.update();
            }

            // Stop all motion;
            robot.mainArm.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }




    private void getColor() throws InterruptedException {

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (robot.cs instanceof SwitchableLight) {
            ((SwitchableLight) robot.cs).enableLight(true);
        }


        // Read the sensor
        colors = robot.cs.getNormalizedColors();


        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        telemetry.addLine()
                .addData("a", "%.3f", colors.alpha)
                .addData("r", "%.3f", colors.red)
                .addData("g", "%.3f", colors.green)
                .addData("g", "%.3f", colors.green)
                .addData("b", "%.3f", colors.blue);

        /* Simplified color sensor code, works a lot better. */
        if (colors.red == colors.blue && colors.red == colors.green && colors.blue == colors.green) { //Monochrome
            telemetry.addData("Color is", "monochrome");
        } else if (colors.red > colors.blue && colors.red > colors.green) { //Color is red
            telemetry.addData("Color is", "red");
        } else if (colors.blue > colors.green && colors.blue > colors.red) { //Color is blue
            telemetry.addData("Color is", "blue");
        } else if (colors.green > colors.blue && colors.green > colors.red) { //Color is green
            telemetry.addData("Color is", "green");
        }


        int color = colors.toColor();
        telemetry.addLine("raw Android color: ").addData("a", "%02x", Color.alpha(color)).addData("r", "%02x", Color.red(color)).addData("g", "%02x", Color.green(color)).addData("b", "%02x", Color.blue(color));
        // Balance the colors. The values returned by getColors() are normalized relative to the
        // maximum possible values that the sensor can measure. For example, a sensor might in a
        // particular configuration be able to internally measure color intensity in a range of
        // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
        // so as to return a value it the range [0,1]. However, and this is the point, even so, the
        // values we see here may not get close to 1.0 in, e.g., low light conditions where the
        // sensor measurements don't approach their maximum limit. In such situations, the *relative*
        // intensities of the colors are likely what is most interesting. Here, for example, we boost
        // the signal on the colors while maintaining their relative balance so as to give more
        // vibrant visual feedback on the robot controller visual display.
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        color = colors.toColor();
        telemetry.addLine("normalized color:  ").addData("a", "%02x", Color.alpha(color)).addData("r", "%02x", Color.red(color)).addData("g", "%02x", Color.green(color)).addData("b", "%02x", Color.blue(color));
        telemetry.update();

        // convert the RGB values to HSV values.
        Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);


    }

    private void setLeft(double power){
        robot.leftDrive.setPower(power);
        robot.leftDrive2.setPower(power);
    }

    private void setRight(double power){
        robot.rightDrive.setPower(power);
        robot.rightDrive2.setPower(power);
    }


}


