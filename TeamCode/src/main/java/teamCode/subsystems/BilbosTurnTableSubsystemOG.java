package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BilbosTurnTableSubsystemOG extends SubsystemBase
{
    private final DcMotor m_turnTableMotor;

    public BilbosTurnTableSubsystemOG(DcMotor slideArmMotor)
    {
        this.m_turnTableMotor = slideArmMotor;
        this.m_turnTableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     public void turnTable(int slide)
    {
        m_turnTableMotor.setTargetPosition(slide);
        this.m_turnTableMotor.setPower(.8);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double setPanPosition()
    {
        return  m_turnTableMotor.getCurrentPosition();
    }

    public double getPanPosition()
    {
        return m_turnTableMotor.getCurrentPosition();
    }
    public void stop()
    {
        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
