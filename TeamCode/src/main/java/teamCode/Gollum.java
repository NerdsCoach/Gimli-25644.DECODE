//package teamCode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//public class Gollum
//{
//        //    private RevColorSensorV3 colorSensor;
//        NormalizedColorSensor m_colorSensor;
//
//        public enum DetectedColor
//        {
//            GREEN,
//            PURPLE,
//            UNKNOWN
//        }
//
//        public Gollum(HardwareMap hardwareMap)
//        {
//            m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//        }
//
//        public DetectedColor getDetectedColor()
//        {
//            NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); //return 4 values
//
//            float normRed, normGreen, normBlue;
//            normRed = colors.red / colors.alpha;
//            normGreen = colors.green / colors.alpha;
//            normBlue = colors.blue / colors.alpha;
//return
//            telemetry.addData("red", normRed);
//            telemetry.addData("green", normGreen);
//            telemetry.addData("blue", normBlue);
//
//            //TODO add if statements for specific colors added
//        }
//    }
//
//
