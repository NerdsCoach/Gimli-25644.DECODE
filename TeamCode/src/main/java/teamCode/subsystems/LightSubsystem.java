package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightSubsystem extends SubsystemBase
{
    private final Servo m_light;
    private static final double GREEN_POS = 0.5;
    private static final double PURPLE_POS = 0.722;
    private static final double OFF_POS = 0.0;

    public LightSubsystem(HardwareMap hMap, String light)
    {
        this.m_light = hMap.get(Servo.class, light);
    }

    public void on (double lightOn )
    {
        m_light.setPosition(lightOn);
    }

    public void off (double lightOff)
    {
        m_light.setPosition(lightOff);
    }

    public void setLEDGreen()
    {
        m_light.setPosition(GREEN_POS);
    }

    public void setLEDPurple()
    {
        m_light.setPosition(PURPLE_POS);
    }

    public void lastKnown (double lastKnown)
    {
        m_light.setPosition(lastKnown);
    }
}
