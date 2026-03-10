package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightASubsystem extends SubsystemBase
{
    private final Servo m_lightA;
    private static final double GREEN_POS = 0.5;
    private static final double PURPLE_POS = 0.722;
    private static final double OFF_POS = 0.0;

    public LightASubsystem(HardwareMap hMap, String light)
    {
        this.m_lightA = hMap.get(Servo.class, light);
    }

    public void on (double lightOn )
    {
        m_lightA.setPosition(lightOn);
    }

    public void off (double lightOff)
    {
        m_lightA.setPosition(lightOff);
    }

    public void setLEDGreen()
    {
        m_lightA.setPosition(GREEN_POS);
    }

    public void setLEDPurple()
    {
        m_lightA.setPosition(PURPLE_POS);
    }

    public void lastKnown (double lastKnown)
    {
        m_lightA.setPosition(lastKnown);
    }
}
