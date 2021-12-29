package org.firstinspires.ftc.teamcode.auto;
import android.graphics.Bitmap;
import android.os.Build;

import androidx.annotation.RequiresApi;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;


public class vision {


    LinearOpMode opMode;
    VuforiaLocalizer vuforia;
    String pos = "notFound";
    int spot1 = 0;
    int spot2 = 0;
    int spot3 = 0;

    public vision(LinearOpMode opMode){
        this.opMode = opMode;
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "AQvLCbX/////AAABmTGnnsC2rUXvp1TAuiOSac0ZMvc3GKI93tFoRn4jPzB3uSMiwj75PNfUU6MaVsNZWczJYOep8LvDeM/3hf1+zO/3w31n1qJTtB2VHle8+MHWNVbNzXKLqfGSdvXK/wYAanXG2PBSKpgO1Fv5Yg27eZfIR7QOh7+J1zT1iKW/VmlsVSSaAzUSzYpfLufQDdE2wWQYrs8ObLq2kC37CeUlJ786gywyHts3Mv12fWCSdTH5oclkaEXsVC/8LxD1m+gpbRc2KC0BXnlwqwA2VqPSFU91vD8eCcD6t2WDbn0oJas31PcooBYWM6UgGm9I2plWazlIok72QG/kOYDh4yXOT4YXp1eYh864e8B7mhM3VclQ";
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
    }


    public Bitmap getImage() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        return bm;
    }

    public String redgetTeamMarkerPos() throws InterruptedException {
        Bitmap rgbImage = getImage();
        pos = "1";
        spot1 = 0;
        spot2 = 0;
        spot3 = 0;
        for(int i = 95; i < 250; i++){
            if(isGreen(rgbImage.getPixel(7,i))) {
                spot3 += 1;
            }
        }
        for(int i = 95; i < 250; i++){
            if(isGreen(rgbImage.getPixel(392,i))) {
                spot2 += 1;
            }
        }
        for(int i = 95; i < 250; i++){
            if(isGreen(rgbImage.getPixel(611,i))) {
                spot1 += 1;
            }
        }
        if(spot1 > spot2 && spot1 > spot3)
            pos = "1";
        else if(spot2 > spot3)
            pos = "2";
        else
            pos = "3";

        opMode.telemetry.addData("spot 1", spot1);
        opMode.telemetry.addData("spot 2", spot2);
        opMode.telemetry.addData("spot 3", spot3);

        return pos;
    }
    public String bluegetTeamMarkerPos() throws InterruptedException {
        int total1 = 0;
        int total2 = 0;
        int total3 = 0;

        pos = "none";
        for(int j = 0; j < 4; j++){
            Bitmap rgbImage = getImage();
            spot1 = 0;
            spot2 = 0;
            spot3 = 0;
            for (int i = 92; i < 222; i++) {
                if (isGreen(rgbImage.getPixel(630, i))) {
                    spot3 += 1;
                }

            }
            for (int i = 92; i < 222; i++) {
                if (isGreen(rgbImage.getPixel(305, i))) {
                    spot2 += 1;
                }
            }
            for (int i = 92; i < 222; i++) {
                if (isGreen(rgbImage.getPixel(24, i))) {
                    spot1 += 1;
                }
            }
            if (spot1 > spot2 && spot1 > spot3)
                total1++;
            else if (spot2 > spot3)
                total2++;
            else
                total3++;
        }
        if (total1 > total2 && total1 > total3)
            pos = "1";
        else if (total2 > total3)
            pos = "2";
        else
            pos = "3";

        opMode.telemetry.addData("spot 1", spot1);
        opMode.telemetry.addData("spot 2", spot2);
        opMode.telemetry.addData("spot 3", spot3);
        opMode.telemetry.addData("spot 1", total1);
        opMode.telemetry.addData("spot 2", total2);
        opMode.telemetry.addData("spot 3", total3);

        return pos;
    }

    public boolean isGreen(int pixel) {
        boolean color = (green(pixel) <= 125) && (green(pixel) >= 70) && (red(pixel) <= 105) && (blue(pixel) <= 140);
        return color;
    }
    @RequiresApi(api = Build.VERSION_CODES.Q)
    public void gettrueColor(int x, int y)throws InterruptedException{

        Bitmap rgbImage = getImage();

        int pixel = rgbImage.getPixel(x, y);
        // w = 640
        // h = 480
        opMode.telemetry.addData("w", rgbImage.getWidth());
        opMode.telemetry.addData("h", rgbImage.getHeight());

        opMode.telemetry.addData("red", red(pixel));
        opMode.telemetry.addData("green", green(pixel));

        opMode.telemetry.addData("blue", blue(pixel));
        opMode.telemetry.addData("Color", rgbImage.getColor(x, y));

        opMode.telemetry.update();

    }

}




