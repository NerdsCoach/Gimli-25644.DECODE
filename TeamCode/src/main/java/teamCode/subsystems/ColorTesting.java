//package teamCode.subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//@TeleOp(name = "AHhhhhhHHH!")
//public class ColorTesting extends CommandOpMode
//{
//    public RevColorSensorV3 m_colorSensor;
//
//    public void initialize(HardwareMap hardwareMap)
//    {
//        m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
//        m_colorSensor.setGain(4);
//    }
//
//    public enum DetectedColor
//    {
//        GREEN,
//        PURPLE,
//        UNKNOWN
//    }
//
//    public ColorTesting getDetectedColor ()
//    {
//        NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); // returns 4 values Alpha is how much light is being returned
//        float normRed;
//        float normGreen;
//        float normBlue;
//        normRed = colors.red / colors.alpha;
//        normGreen = colors.green / colors.alpha;
//        normBlue = colors.blue / colors.alpha; // so you always get the same reading at different distances
//
//        telemetry.addData("red", normRed);
//        telemetry.addData("green", normGreen);
//        telemetry.addData("blue", normBlue);
//
//        //TODO Add if statements for each color
//
//       // Green Ball
//        //Red=
//        //Green=
//        //Blue=
//
//        //Purple Ball
//         //Red=
//         //Green=
//         //Blue=
//
//        return ColorTesting.DetectedColor.UNKNOWN;
//
//    }
//}
