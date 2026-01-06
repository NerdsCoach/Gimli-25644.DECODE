package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServoSubsystem extends SubsystemBase
{
    private final Servo m_launcherServo;

    public HoodServoSubsystem(HardwareMap hMap, String launcherServo )
    {
        this.m_launcherServo = hMap.get(Servo.class, launcherServo);
    }

    public void pivotHood(double launcherPos)
    {
        this.m_launcherServo.setPosition(launcherPos);
    }
}
