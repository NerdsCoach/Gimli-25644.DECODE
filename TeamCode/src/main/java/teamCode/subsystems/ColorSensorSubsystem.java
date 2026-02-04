package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorSensorSubsystem extends SubsystemBase
{
    public NormalizedColorSensor m_colorSensor;
    private final Servo m_light;


        // Define the sensor and LED driver

        // Default Servo positions for goBILDA Prism colors (examples)
        private static final double GREEN_POS = 0.5;
        private static final double PURPLE_POS = 0.722;
        private static final double OFF_POS = 0.0;


    public ColorSensorSubsystem(HardwareMap hardwareMap)
        {
            // Initialize from hardware map
            m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
            m_light = hardwareMap.get(Servo.class, "light");

            // Adjust gain for better low-light sensitivity
            m_colorSensor.setGain(2.0f);
        }

        public NormalizedRGBA getNormalizedColors()
        {
            return m_colorSensor.getNormalizedColors();
        }

        public void setLEDGreen()
        {
            m_light.setPosition(GREEN_POS);
        }
        public void setLEDPurple()
        {
            m_light.setPosition(PURPLE_POS);
        }
        public void setLEDOff()
        {
            m_light.setPosition(OFF_POS);
        }
        public void lastKnown (double lastKnown)
        {
            m_light.setPosition(lastKnown);
        }

}
