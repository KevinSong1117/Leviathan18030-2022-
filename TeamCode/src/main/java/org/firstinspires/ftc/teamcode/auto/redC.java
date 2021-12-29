package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.teamcode.auto.vision;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.green;

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

@Autonomous(name="redC", group="redC")

public class redC extends LinearOpMode
{
    // Declare OpMode members.
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right
    public DcMotor ER;  // lift extend right
    public DcMotor EL;  // lift extend left
    public CRServo IR;
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left
    public BNO055IMU imu;
    private vision vision;
    Orientation angles;
    float curHeading;
    public vision v;
    LinearOpMode opMode;
    ElapsedTime timer;
    Sensors gyro;
    public DcMotor DG;


    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException  {
        //throw new UnsupportedOperationException();
        timer = new ElapsedTime();
        vision = new vision(this);
        fL = hardwareMap.get(DcMotor.class, "FL");
        fR = hardwareMap.get(DcMotor.class, "FR");
        bL = hardwareMap.get(DcMotor.class, "BL");
        bR = hardwareMap.get(DcMotor.class, "BR");

        ER = hardwareMap.get(DcMotor.class, "ER");

        IR = hardwareMap.get(CRServo.class, "IR");
        WR = hardwareMap.get(CRServo.class, "WR");
        WL = hardwareMap.get(CRServo.class, "WL");
        gyro = new Sensors(this);
        DG = hardwareMap.get(DcMotor.class, "DG");
        DG.setDirection((DcMotorSimple.Direction.FORWARD));
        DG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IR.setDirection(CRServo.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        ER.setDirection(DcMotor.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        String position = vision.redgetTeamMarkerPos();

        while(!isStarted()){
            position = vision.redgetTeamMarkerPos();
            telemetry.addData("redposition", position);
            telemetry.update();

        }

        waitForStart();

        moveForward(200, -.5);
        turn(120, .5);
        deliverA("3");

        //String position = v.getTeamMarkerPos();

        // see if the team element is in the 3 different positions
        // if the camera dose not detect the team element it will only do other tasks
        /*if(position.equals("1")){
            telemetry.addData("pos", position);
            moveForward(500, .5);
        }

        else if(position.equals("2")){
            telemetry.addData("pos", position);
            moveForward(500, .5);
        }

        else if(position.equals("3")){
            telemetry.addData("pos", position);
            moveForward(500, .5);
        }

        else{
            telemetry.addData("pos", position);
        }*/
    }

    public void spinDucks(double power, long time) { // Sets power to rubber duck spinner for a set amount of time and then stops
        DG.setPower(power);
        sleep(time);
        DG.setPower(0);
    }

    public void moveForward(double tics, double power) {
        while (!isStopRequested() && opModeIsActive()) {
            resetEncoder();
            while (getTic() < tics) {
                fL.setPower(power);              // encoding start is less than the target
                fR.setPower(power); // Sets the power of the motors to currently half the power
                bL.setPower(power);
                bR.setPower(power);
            }
            stopMotors();
            break;
        }
    }
    public void lift(long height){ //Lifts the arm up to height value which is the milisec amount for sleep()
        ER.setPower(-.1 + (.7));
        sleep(height);
        ER.setPower(-.2);
    }
    public void deliver(){  // Sets the power to outtake wheels fo 3 seconds and stops them
        IR.setPower(.5);
        sleep(3000);
        IR.setPower(0);
    }
    public void down(){ //Sets power so that arm slowly goes down
        ER.setPower(-.0005 + (-.1 * .35));
    }
    public void deliverA(String level){
        if(level.equals("1")){
            lift(300);
            WR.setPower(-.5);
            WL.setPower(-.5);
            moveForward(300, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            turn(150,.5);
            turn(40,.5);
            moveForward(500, .5);
            spinDucks(.4,3500);
            moveForward(500, -.5);
            turn(-40,.5);
            moveForward(500,5.);
        }
        else if(level.equals("2")){
            lift(700);
            WR.setPower(-.5);
            WL.setPower(-.5);
            moveForward(600, .5);
            deliver();
            moveForward(600, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            turn(150,.5);
            turn(40,.5);
            moveForward(500, .5);
            spinDucks(.4,3500);
            moveForward(500, -.5);
            turn(-40,.5);
            moveForward(500,5.);
        }
        else{
            lift(500);
            WR.setPower(-.5);
            WL.setPower(-.5);
            moveForward(900, .5);
            deliver();
            moveForward(900, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            turn(150,.5);
            turn(40,.5);
            moveForward(500, .5);
            spinDucks(.4,3500);
            moveForward(500, -.5);
            turn(-40,.5);
            moveForward(500,5.);
        }
    }

    public void turn(double degree, double power){
        while (opModeIsActive() && !isStopRequested()) {
            if (angleWrapDeg(degree - gyro.getAngle()) > 0) {
                while (angleWrapDeg(degree - gyro.getAngle()) > 0) {
                    fL.setPower(-power);
                    fR.setPower(power);
                    bL.setPower(-power);
                    bR.setPower(power);
                }

            } else {
                while (angleWrapDeg(degree - gyro.getAngle()) < 0) {
                    fL.setPower(power);
                    fR.setPower(-power);
                    bL.setPower(power);
                    bR.setPower(-power);
                }
            }
            stopMotors();
            break;
        }
    }

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


    public void resetEncoder() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void startMotors(double right, double left) {
        fR.setPower(right);
        fL.setPower(left);
        bL.setPower(left);
        bR.setPower(right);
        telemetry.addData("right power", right);
        telemetry.addData("left power", left);
        telemetry.update();
    }
    public void stopMotors() {
        fR.setPower(0);
        fL.setPower(0);
        bR.setPower(0);
        bL.setPower(0);
    }
    public double getTic() {
        double count = 4;
        if (fR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (fL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = Math.abs(fR.getCurrentPosition()) + Math.abs(fL.getCurrentPosition()) + Math.abs(bL.getCurrentPosition()) + Math.abs(bR.getCurrentPosition());
        if (count == 0) {
            return 1;
        }
        return totaldis / count;
    }
    double angleWrapDeg(double angle) {
        double correctAngle = angle;
        while (correctAngle > 180)
        {
            correctAngle -= 360;
        }
        while (correctAngle < -180)
        {
            correctAngle += 360;
        }
        return correctAngle;
    }

    /* public void movePIDFGyro(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;


        while (timeAtSetPoint < time) {
            if (inches < 0){
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;

            double difference = gyro.angleDiff(initialHeading);

            if (difference > .4){
                if (power > 0) {
                    startMotors((power + f), (power + f) * .8);
                }
                else {
                    startMotors((power - f) * .8, (power - f));
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    startMotors((power + f), (power + f));
                }
                else {
                    startMotors((power - f),(power - f));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f,  power + f);
                }
                else {
                    startMotors(power - f,power - f);
                }
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }
    public void turnHeading(double finalAngle, double kp, double ki, double kd, double f, double threshold, double time) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleWrapDeg(finalAngle - initialHeading);
        double error = initialAngleDiff;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (timeAtSetPoint < time) {
            error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / Math.abs(initialAngleDiff);
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            if (power > 0) {
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors(-power - f, power + f);
            }
            else{
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors(-power + f, power - f);
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }

     */
}
