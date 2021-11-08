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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="teleop", group="teleop")

public  class test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right
    public DcMotor ER;  // lift extend right
    public DcMotor EL;  // lift extend left
    public CRServo IR;
    public CRServo IL;
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "FL");
        fR = hardwareMap.get(DcMotor.class, "FR");
        bL = hardwareMap.get(DcMotor.class, "BL");
        bR = hardwareMap.get(DcMotor.class, "BR");
        LTL = hardwareMap.get(DcMotor.class, "LTL");
        LTR = hardwareMap.get(DcMotor.class, "LTR");
        ER = hardwareMap.get(DcMotor.class, "EL");
        EL = hardwareMap.get(DcMotor.class, "ER");

        IR = hardwareMap.get(CRServo.class, "IR");
        IL = hardwareMap.get(CRServo.class, "IL");
        WR = hardwareMap.get(CRServo.class, "IR");
        WL = hardwareMap.get(CRServo.class, "IL");

        IL.setDirection(CRServo.Direction.FORWARD);
        IR.setDirection(CRServo.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        EL.setDirection(DcMotor.Direction.FORWARD);
        ER.setDirection(DcMotor.Direction.REVERSE);
        LTR.setDirection(DcMotor.Direction.FORWARD);
        LTL.setDirection(DcMotor.Direction.REVERSE);


        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double limitPower = .8; // percent of power
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        if (leftPower < 0.1 || rightPower < 0.1){
            fL.setPower(0);
            fR.setPower(0); // Sets the power of the motors to zero when the power is small
            bL.setPower(0);
            bR.setPower(0);
        }

        else {
            fL.setPower(leftPower * limitPower);
            fR.setPower(rightPower * limitPower); // Sets the power of the motors to currently half the power
            bL.setPower(leftPower * limitPower);
            bR.setPower(rightPower * limitPower);
        }

        telemetry.addData("Left: ", leftPower);
        telemetry.addData("Right: ", rightPower);
        telemetry.update();
        telemetry.update();
        // tests encoder
        telemetry.addData("fl", fL.getCurrentPosition());
        telemetry.addData("fr", fR.getCurrentPosition());
        telemetry.addData("bl", bL.getCurrentPosition());
        telemetry.addData("br", bR.getCurrentPosition());
        telemetry.update();
        double power = 0;
        if(gamepad2.a){
            power = .5;
            telemetry.addData("power: ", power);
            IR.setPower(power);
            IL.setPower(power);

        }
        if(gamepad2.b){
            power = -.5;
            telemetry.addData("power: ", power);
            IR.setPower(power);
            IL.setPower(power);
        }

        if(gamepad2.x) {
            WR.setPower(.1);
            WL.setPower(.1);
        }
        if(gamepad2.y) {
            WR.setPower(-.1);
            WL.setPower(-.1);
        }
        double extendPower = gamepad2.left_stick_y;
        double armPosition = gamepad2.right_stick_x;
        // test this for extending lift
        ER.setPower(extendPower);
        // moving the right motor towards the front of robot = retract
        // moving the left motor towards the back of the robot = extend
        EL.setPower(-extendPower);

        LTR.setPower(armPosition);
        LTL.setPower(armPosition);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
