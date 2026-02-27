package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeMotorSubsystem extends SubsystemBase
{
    private final DcMotor m_intakeMotor;

    public IntakeMotorSubsystem(DcMotor intakeMotor)
    {
        this.m_intakeMotor = intakeMotor;
        this.m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.m_intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void spinMotorIntake(double speed)
    {
        this.m_intakeMotor.setPower(speed);
    }

    public void stop()
    {
        this.m_intakeMotor.setPower(0);
    }
}
