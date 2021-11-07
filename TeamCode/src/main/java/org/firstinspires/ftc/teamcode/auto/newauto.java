package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@Autonomous(name="auto", group="newauto")

public class newauto extends LinearOpMode
{
    // Declare OpMode members.
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;  // instantiates motor variables
    public DcMotor BR;
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right
    public DcMotor ER;  // lift extend right
    public DcMotor EL;  // lift extend left
    public CRServo IR;
    public CRServo IL;
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left
    public BNO055IMU imu;
    Orientation angles;
    float curHeading;
    public vision v;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void runOpmode(LinearOpMode lOpmode) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        LTL = hardwareMap.get(DcMotor.class, "LTL");
        LTR = hardwareMap.get(DcMotor.class, "LTR");
        ER = hardwareMap.get(DcMotor.class, "EL");
        EL = hardwareMap.get(DcMotor.class, "ER");

        IR = hardwareMap.get(CRServo.class, "IR");
        IL = hardwareMap.get(CRServo.class, "IL");

        IL.setDirection(CRServo.Direction.FORWARD);
        IR.setDirection(CRServo.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        imu = lOpmode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        v = new vision(this);

    }


    public double getEncoderAvg(){
        double flEncoder = FL.getCurrentPosition();
        double frEncoder = FR.getCurrentPosition();
        double blEncoder = BL.getCurrentPosition();
        double brEncoder = BR.getCurrentPosition();

        double ret = flEncoder + frEncoder + blEncoder + brEncoder;
        ret /= 4;

        return ret;
    }
    private void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle; //Gets the orientation of the robot
    }

    public void moveForBack(double distance, double direction){    //takes two variables, one for the direction
        // goal and one for the distance traveled
        double sEncoder = getEncoderAvg();

        while(getEncoderAvg() - sEncoder < distance){ // Runs as long as the encoding average - the
            FL.setPower(direction * .5);              // encoding start is less than the target
            FR.setPower(direction * .5); // Sets the power of the motors to currently half the power
            BL.setPower(direction * .5);
            BR.setPower(direction * .5);
        }
        FL.setPower(0);
        FR.setPower(0); // Sets the power of the motors to currently half the power
        BL.setPower(0);
        BR.setPower(0);
    }
    public void botTurning( boolean direction, float degree) { //direction is to know if it will
        // turn left or right, degree is to know the amount which it turns. Positive is right
        checkOrientation();
        float start = curHeading;// Gets the current heading

        if (direction) {
            while (curHeading > start - degree) { // turns left
                FL.setPower(.5);
                FR.setPower(-.5);
                BL.setPower(.5);
                BR.setPower(-.5);
                checkOrientation();
            }
        }
        else {
            while (curHeading < start + degree) { // turns right
                FL.setPower(-.5);
                FR.setPower(.5);
                BL.setPower(-.5);
                BR.setPower(.5);
                checkOrientation();
            }
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    public void carousel(){
        IL.setPower(1);
        IR.setPower(-1);

    }

}
