package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TurnTableSubsystem extends SubsystemBase
{
    private final DcMotor m_turnTableMotor;

    public TurnTableSubsystem(DcMotor turnTableMotor)
    {
        this.m_turnTableMotor = turnTableMotor;
        this.m_turnTableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     public void turnTable(int turn)
    {
        this.m_turnTableMotor.setTargetPosition(this.m_turnTableMotor.getCurrentPosition() + turn);

//        this.m_turnTableMotor.setTargetPosition(turn);
        this.m_turnTableMotor.setPower(.8);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
//    public void fudgeFactor(int turn)
//    {
//        this.m_turnTableMotor.setTargetPosition(this.m_turnTableMotor.getCurrentPosition() + turn);
//        this.m_turnTableMotor.setPower(0.75);
//        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    public boolean atTarget(double target)
    {
        return this.m_turnTableMotor.getCurrentPosition() <= target+5 && this.m_turnTableMotor.getCurrentPosition() >= target-5;
    }

//    public boolean atTarget(double target)
//    {
//        return this.m_turnTableMotor.getCurrentPosition() <= target+5 && this.m_turnTableMotor.getCurrentPosition() >= target-5;
//    }

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
        this.m_turnTableMotor.setPower(0);
        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
