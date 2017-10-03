package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@Autonomous(name="Motor Test", group="Cardbot")
public class CardbotMotorTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCardbot         robot   = new HardwareCardbot();   // Use a Pushbot's hardware



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        long s = 1000;
        telemetry.addData("Say", "Starting");
        telemetry.update();

        telemetry.addData("Say", "ld");
        telemetry.update();
        robot.leftDrive.setPower(1);
        try {
            Thread.sleep(5 * s);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.leftDrive.setPower(0);

        telemetry.addData("Say", "ld2");
        telemetry.update();
        robot.leftDrive2.setPower(1);
        try {
            Thread.sleep(5 * s);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.leftDrive2.setPower(0);

        telemetry.addData("Say", "rd");
        telemetry.update();
        robot.rightDrive.setPower(1);
        try {
            Thread.sleep(5 * s);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.rightDrive.setPower(0);

        telemetry.addData("Say", "rd2");
        telemetry.update();
        robot.rightDrive2.setPower(1);
        try {
            Thread.sleep(5 * s);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.rightDrive2.setPower(0);
    }

}
