package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightBSubsystem extends SubsystemBase
{
    private final Servo m_lightB;
    private static final double GREEN_POS = 0.5;
    private static final double PURPLE_POS = 0.722;
    private static final double OFF_POS = 0.0;

    public LightBSubsystem(HardwareMap hMap, String light)
    {
        this.m_lightB = hMap.get(Servo.class, light);
    }

    public void on (double lightOn )
    {
        m_lightB.setPosition(lightOn);
    }

    public void off (double lightOff)
    {
        m_lightB.setPosition(lightOff);
    }

    public void setLEDGreen()
    {
        m_lightB.setPosition(GREEN_POS);
    }

    public void setLEDPurple()
    {
        m_lightB.setPosition(PURPLE_POS);
    }

    public void lastKnown (double lastKnown)
    {
        m_lightB.setPosition(lastKnown);
    }
}
