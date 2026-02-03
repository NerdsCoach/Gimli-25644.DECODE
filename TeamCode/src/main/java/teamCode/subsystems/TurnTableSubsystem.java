package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnTableSubsystem extends SubsystemBase
{
        private final DcMotor m_turnTableMotor;
        private static final double TURRET_SPEED = 1.0;//making it faster

    public TurnTableSubsystem(DcMotor turnTableMotor)
    {
        m_turnTableMotor = turnTableMotor;
        m_turnTableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnSpeed(double speed)
    {
        m_turnTableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_turnTableMotor.setPower(speed * TURRET_SPEED);
    }

    public void stop()
    {
        m_turnTableMotor.setPower(0.0);
    }

    public void newTurnTable (int pull)
    {
        this.m_turnTableMotor.setTargetPosition(this.m_turnTableMotor.getCurrentPosition() + pull);
        this.m_turnTableMotor.setPower(0.75);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Turn (int target)
    {
        this.m_turnTableMotor.setTargetPosition(target);
        this.m_turnTableMotor.setPower(0.3);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTurnPower(double power)
    {
        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.m_turnTableMotor.setPower(power);
    }

    public boolean atTargetLeft(double targetLeft)
    {
        return this.m_turnTableMotor.getCurrentPosition() >= targetLeft-5 && this.m_turnTableMotor.getCurrentPosition() <= targetLeft+5;
    }

    public boolean atTargetRight (double targetRight)
    {
        return this.m_turnTableMotor.getCurrentPosition() >= targetRight-5 && this.m_turnTableMotor.getCurrentPosition() <= targetRight+5;
    }

    public int getCurrentPosition()
    {
        return m_turnTableMotor.getCurrentPosition();
    }
}
