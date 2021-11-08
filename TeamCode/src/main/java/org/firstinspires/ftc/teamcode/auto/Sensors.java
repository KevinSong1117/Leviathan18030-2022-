
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class Sensors
{
    BNO055IMU imu;
    Orientation angles;

    public Sensors(LinearOpMode opMode){
        opMode.telemetry.addLine("initializing imu, please wait");
        opMode.telemetry.speak("initializing imu, please wait");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        boolean initialize = imu.initialize(parameters);
        opMode.telemetry.addLine("initialization complete");
        opMode.telemetry.speak("initialization complete");
    }

    public Sensors(OpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void updateAngle() {
        angles = imu.getAngularOrientation();
    }

    public double getAngle() {
        updateAngle();
        return angles.firstAngle;
    }

    public double newAngleDiff(double angle1, double angle2)
    {
        if (angle1 >= 0 && angle2 >= 0 || angle1 <= 0 && angle2 <= 0)
        { //curr & goal are both positive or both negative
            return -(angle1 - angle2);
        }
        else if (Math.abs(angle1 - angle2) <= 180)
        { //diff btwn curr & goal is less than or equal to 180
            return -(angle1 - angle2);
        }
        else if (angle1 > angle2)
        { //curr is greater than goal
            return (360 - (angle1 - angle2));
        }
        else
        { //goal is greater than curr
            return -(360 + (angle1 - angle2));
        }
    }

    public double angleDiff(double goalAngle) {
        double currAngle = getAngle();
        if (currAngle >= 0 && goalAngle >= 0 || currAngle <= 0 && goalAngle <= 0) { //curr & goal are both positive or both negative
            return -(currAngle - goalAngle);
        } else if (Math.abs(currAngle - goalAngle) <= 180) {//diff btwn curr & goal is less than or equal to 180
            return -(currAngle - goalAngle);
        } else if (currAngle > goalAngle) {//curr is greater than goal
            return (360 - (currAngle - goalAngle));
        } else {//goal is greater than curr
            return -(360 + (currAngle - goalAngle));
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
