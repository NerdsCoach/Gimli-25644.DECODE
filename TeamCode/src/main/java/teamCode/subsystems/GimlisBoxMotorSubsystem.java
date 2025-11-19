package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class GimlisBoxMotorSubsystem extends SubsystemBase
{
    private final DcMotor m_parkMotor;

    public GimlisBoxMotorSubsystem(DcMotor parkMotor)
    {
        this.m_parkMotor = parkMotor;
        this.m_parkMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.m_parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_parkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     public void liftPark(int lift)
    {
        m_parkMotor.setTargetPosition(lift);
        this.m_parkMotor.setPower(.8);
        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop()
    {
        this.m_parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
