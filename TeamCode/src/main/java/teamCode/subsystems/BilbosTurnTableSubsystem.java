package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BilbosTurnTableSubsystem extends SubsystemBase
{
    private final DcMotor m_turntableMotor;

    public BilbosTurnTableSubsystem (DcMotor turntableMotor)
    {
        this.m_turntableMotor = turntableMotor;
        this.m_turntableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.m_turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void turntable (int turntable)
    {
        this.m_turntableMotor.setTargetPosition(turntable);
        this.m_turntableMotor.setPower(0.8);
        this.m_turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stop()
    {
        this.m_turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
