package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferServoSubsystem extends SubsystemBase
{
    private final Servo m_transferServo;

    public TransferServoSubsystem(HardwareMap hMap, String transferServo)
    {
        this.m_transferServo = hMap.get(Servo.class, transferServo);
    }

    // Pivots position of the intake.
    public void transfer (double transferPos)
    {
        this.m_transferServo.setPosition(transferPos);
    }
    public boolean atTarget(double target)
    {
        return this.m_transferServo.getPosition() >= target-.01 && this.m_transferServo.getPosition() <= target+.01;
    }
}
