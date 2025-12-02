package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightSubsystem extends SubsystemBase
{
    private final Servo m_light;

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
}
