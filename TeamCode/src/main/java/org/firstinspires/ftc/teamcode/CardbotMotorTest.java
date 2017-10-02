package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@Autonomous(name="Motor Test", group="Cardbot")
@Disabled
public class CardbotMotorTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCardbot         robot   = new HardwareCardbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        long s = 1000;
        try {
            robot.leftDrive.setPower(1);
            Thread.sleep(5 * s);
            telemetry.addData("Say", "ld");
            telemetry.update();
            robot.leftDrive.setPower(0);

            robot.leftDrive2.setPower(1);
            Thread.sleep(5 * s);
            telemetry.addData("Say", "ld2");
            telemetry.update();
            robot.leftDrive2.setPower(0);

            robot.rightDrive.setPower(1);
            Thread.sleep(5 * s);
            telemetry.addData("Say", "rd");
            telemetry.update();
            robot.rightDrive.setPower(0);

            robot.rightDrive2.setPower(1);
            Thread.sleep(5 * s);
            telemetry.addData("Say", "rd2");
            telemetry.update();
            robot.rightDrive2.setPower(0);
        } catch(Exception e) {}
    }

}
