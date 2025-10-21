package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GondorsDefenseTransferSubsystem extends SubsystemBase
{
    private final Servo m_transferServo;

    public GondorsDefenseTransferSubsystem(HardwareMap hMap, String name)
    {
        this.m_transferServo = hMap.get(Servo.class, name);
    }

    // Pivots position of the intake.
    public void pivotServo(double pos)
    {
        this.m_transferServo.setPosition(pos);
    }
}
