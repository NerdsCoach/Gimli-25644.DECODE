package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ParkingSubsystem extends SubsystemBase
{
    private final DcMotor m_parkMotor;

    public ParkingSubsystem(DcMotor parkMotor)
    {
        this.m_parkMotor = parkMotor;
        this.m_parkMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.m_parkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void parking(int park)
    {
        m_parkMotor.setTargetPosition(park);
        this.m_parkMotor.setPower(0.75);
        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void parkFudgeFactor(int up)
    {
        this.m_parkMotor.setTargetPosition(this.m_parkMotor.getCurrentPosition() + up);
        this.m_parkMotor.setPower(0.75);
        this.m_parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean atTargetUp(double targetUp)
    {
        return this.m_parkMotor.getCurrentPosition() >= targetUp;
    }

    public boolean atTargetDown (double targetDown)
    {
        return this.m_parkMotor.getCurrentPosition() <= targetDown;
    }

}
