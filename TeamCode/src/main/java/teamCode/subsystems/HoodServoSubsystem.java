package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServoSubsystem extends SubsystemBase
{
    private final Servo m_aimingServo;

    public HoodServoSubsystem(HardwareMap hMap, String aimingServo)
    {
        this.m_aimingServo = hMap.get(Servo.class, aimingServo);
    }

    public void pivotHood(double movePos)
    {
        this.m_aimingServo.setPosition(movePos);
    }
}
