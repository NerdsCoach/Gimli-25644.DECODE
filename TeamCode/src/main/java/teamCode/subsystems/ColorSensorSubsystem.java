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

}
