/*
*   Copyright (c) 2025 Alan Smith
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:

*   The above copyright notice and this permission notice shall be included in all
*   copies or substantial portions of the Software.

*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/
package teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ColorTesting")
public class ColorTesting extends LinearOpMode
{
    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.update();
        }
    }
}
//public class ColorTesting extends OpMode
//{
//    NormalizedColorSensor m_colorSensor;
//
//    public enum DetectedColor
//    {
//        GREEN,
//        PURPLE,
//        UNKNOWN
//    }
//    public ColorTesting(HardwareMap hardwareMap)
//    {
//        m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//    }
//
//    public teamCode.subsystems.GollumsPreciousColorSensorSubsystem.DetectedColor getDetectedColor()
//    {
//        NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); //return 4 values
//
//        float normRed, normGreen, normBlue;
//        normRed = colors.red / colors.alpha;
//        normGreen = colors.green / colors.alpha;
//        normBlue = colors.blue / colors.alpha;
//
//        telemetry.addData("red", normRed);
//        telemetry.addData("green", normGreen);
//        telemetry.addData("blue", normBlue);
//
//        //TODO add if statements for specific colors added
//        return teamCode.subsystems.GollumsPreciousColorSensorSubsystem.DetectedColor.UNKNOWN;
//    }
//
//    @Override
//    public void init()
//    {
//        m_colorSensor.getNormalizedColors();
//    }
//
//    @Override
//    public void loop()
//    {
//        m_colorSensor.col
//    }
//
//    public void configurePinpoint()
//    {
//
//    }

