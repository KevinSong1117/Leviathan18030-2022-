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

@Autonomous(name="auto", group="auto")

public class auto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    public BNO055IMU imu;
    Orientation angles;
    float curHeading;
    public CRServo inLeft;
    public CRServo inRight;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init(LinearOpMode lOpmode) {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        inLeft = hardwareMap.get(CRServo.class, "inLeft Servo");
        inRight = hardwareMap.get(CRServo.class, "inRight Servo");

        inLeft.setDirection(CRServo.Direction.FORWARD);
        inRight.setDirection(CRServo.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);

        imu = lOpmode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

    }

    @Override
    public void init() {

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
    public double getEncoderAvg(){
        double flEncoder = fL.getCurrentPosition();
        double frEncoder = fR.getCurrentPosition();
        double blEncoder = bL.getCurrentPosition();
        double brEncoder = bR.getCurrentPosition();

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
            fL.setPower(direction * .5);              // encoding start is less than the target
            fR.setPower(direction * .5); // Sets the power of the motors to currently half the power
            bL.setPower(direction * .5);
            bR.setPower(direction * .5);
        }
        fL.setPower(0);
        fR.setPower(0); // Sets the power of the motors to currently half the power
        bL.setPower(0);
        bR.setPower(0);
    }
    public void botTurning( boolean direction, float degree) { //direction is to know if it will
        // turn left or right, degree is to know the amount which it turns. Positive is right
        checkOrientation();
        float start = curHeading;// Gets the current heading

        if (direction) {
            while (curHeading > start - degree) { // turns left
                fL.setPower(.5);
                fR.setPower(-.5);
                bL.setPower(.5);
                bR.setPower(-.5);
                checkOrientation();
            }
        }
        else {
            while (curHeading < start + degree) { // turns right
                fL.setPower(-.5);
                fR.setPower(.5);
                bL.setPower(-.5);
                bR.setPower(.5);
                checkOrientation();
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    public void carousel(){
        inLeft.setPower(1);
        inRight.setPower(-1);

    }
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
