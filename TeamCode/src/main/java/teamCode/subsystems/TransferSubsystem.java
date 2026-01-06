package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class TransferSubsystem extends SubsystemBase
{
    private final CRServo m_transferServo;

    public TransferSubsystem(CRServo transfer)
    {
        this.m_transferServo = transfer;
        this.m_transferServo.setRunMode(Motor.RunMode.VelocityControl);
    }
    public void spinTransfer(double speed)
    {
        this.m_transferServo.set(speed);
    }
//    public void spinTransfer(int spin)
//    {
//        this.m_transferServo.setTargetPosition(spin);
//    }

}




