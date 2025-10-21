//package teamCode.subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import android.view.View;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class GollumsPreciousColorSensorSubsystem extends SubsystemBase
//{
//    //    private RevColorSensorV3 colorSensor;
//    NormalizedColorSensor m_colorSensor;
//
//    public enum DetectedColor
//    {
//        GREEN,
//        PURPLE,
//        UNKNOWN
//    }
//
//    public GollumsPreciousColorSensorSubsystem(HardwareMap hardwareMap)
//    {
//        m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//    }
//
//    public DetectedColor getDetectedColor() {
//        NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); //return 4 values
//
//        float normRed, normGreen, normBlue;
//        normRed = colors.red / colors.alpha;
//        normGreen = colors.green / colors.alpha;
//        normBlue = colors.blue / colors.alpha;
//
//    }
//    }
//}
