package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AxeSubsystem extends SubsystemBase
{
    private final Servo m_transferServo;

    public AxeSubsystem(HardwareMap hMap, String transferServo)
    {
        this.m_transferServo = hMap.get(Servo.class, transferServo);
    }

    public void pivotAxe (double transferPos)
    {
        this.m_transferServo.setPosition(transferPos);
    }
}


