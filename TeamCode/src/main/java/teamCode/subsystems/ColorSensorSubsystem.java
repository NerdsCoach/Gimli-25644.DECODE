package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorSensorSubsystem extends SubsystemBase
{
    public NormalizedColorSensor m_colorSensor;
    private static final double GREEN_POS = 0.5;
    private static final double PURPLE_POS = 0.722;
    private static final double OFF_POS = 0.0;


    public ColorSensorSubsystem(HardwareMap hardwareMap)
        {
            m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
            // Adjust gain for better low-light sensitivity
            m_colorSensor.setGain(2.0f);
        }

        public NormalizedRGBA getNormalizedColors()
        {
            return m_colorSensor.getNormalizedColors();
        }

    public String getDetectedColor() {
        NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); // Use your sensor variable name
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        float hue = hsv[0];
        float saturation = hsv[1];

        // 1. Check if a ball is even there (Low saturation = no ball)
        if (saturation < 0.3) return "NONE";

        // 2. Check Hue (Based on your Command's values)
        if (Math.abs(hue - 160.0f) < 20.0f) return "GREEN";
        if (Math.abs(hue - 240.0f) < 20.0f) return "PURPLE";

        return "NONE";
    }







}
