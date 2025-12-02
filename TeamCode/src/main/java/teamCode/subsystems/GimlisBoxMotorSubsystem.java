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
        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.m_parkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinIntake(double park)
    {
        this.m_parkMotor.setPower(park);
    }
    public void spinIntake(int dePark)
    {
        this.m_parkMotor.setTargetPosition(dePark);
    }

//     public void Park(int target)
//    {
//        this.m_parkMotor.setTargetPosition(target);
//        this.m_parkMotor.setPower(-.7);
//        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void dePark()
//    {
//        this.m_parkMotor.setPower(0.7);
//        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.m_parkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void fudgeFactor(int fudge)
//    {
//        this.m_parkMotor.setTargetPosition(this.m_parkMotor.getCurrentPosition() + fudge);
//        this.m_parkMotor.setPower(0.75);
//        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public boolean atTarget(int target)
//    {
//        return this.m_parkMotor.getCurrentPosition() >= target-5 && this.m_parkMotor.getCurrentPosition() <= target+5;
//    }
//
//    public void stop()
//    {
//        this.m_parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
}
