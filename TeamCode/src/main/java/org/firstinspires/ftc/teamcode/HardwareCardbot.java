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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class HardwareCardbot
{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDrive2  = null;
    public DcMotor  rightDrive2 = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;




    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareCardbot(){}


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;


        leftDrive  = hwMap.get(DcMotor.class, "ld"); // Left Drive
        rightDrive = hwMap.get(DcMotor.class, "rd"); // Right Drive
        leftDrive2 = hwMap.get(DcMotor.class, "ld2"); // Left Drive 2
        rightDrive2= hwMap.get(DcMotor.class, "rd2"); // Right Drive 2
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // DEF: FORWARD
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// DEF: REVERSE
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);
        reverse(leftDrive);
        reverse(leftDrive2);

        leftClaw = hwMap.get(Servo.class, "lc");
        rightClaw = hwMap.get(Servo.class, "rc");


        /*       * ROBOT OUTLINE *
         *      * Outlines Config *
         *   ld  <- :---------: -> rd
         *          |---------|
         *          |---------|
         *          |-BTRY----|
         *   ld2 <- :--REVHB--: -> rd2
         *
         */


        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);


        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftClaw.setPosition(0);
        rightClaw.setPosition(1);


    }


    public void reverse(DcMotor motorIn) {
        Direction motorCurDir = motorIn.getDirection();
        if(motorCurDir == Direction.FORWARD) {
            motorIn.setDirection(Direction.REVERSE);
        } else {
            motorIn.setDirection(Direction.FORWARD);
        }
    }

 }

