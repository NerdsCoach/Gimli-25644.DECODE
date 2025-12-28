package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LauncherMotorSubsystem extends SubsystemBase
{
    private final DcMotor m_launcherMotorRed;
//    private final DcMotor m_launcherMotorBlue;

    public LauncherMotorSubsystem(DcMotor launcherMotorRed/*, DcMotor launcherMotorBlue*/)
    {
       this.m_launcherMotorRed = launcherMotorRed;
       this.m_launcherMotorRed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       this.m_launcherMotorRed.setDirection(DcMotorSimple.Direction.FORWARD);
       this.m_launcherMotorRed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//
//       this.m_launcherMotorBlue = launcherMotorBlue;
//       this.m_launcherMotorBlue.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       this.m_launcherMotorBlue.setDirection(DcMotorSimple.Direction.FORWARD );
//       this.m_launcherMotorBlue.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void setSafePower(DcMotor launcherMotorRed, double targetPowerRed, DcMotor launcherMotorBlue, double targetPowerBlue)
//    {
//        final double SLEW_RATE_RED = 0.8;
//        double currentPowerRed = launcherMotorRed.getPower();
//        double desiredChangeRed = targetPowerRed - currentPowerRed;
//        double limitedChangeRed = Math.max(-SLEW_RATE_RED, Math.min(desiredChangeRed, SLEW_RATE_RED));
//        launcherMotorRed.setPower(currentPowerRed += limitedChangeRed);
//
//        final double SLEW_RATE_BLUE = 0.8;
//        double currentPowerBlue = launcherMotorBlue.getPower();
//        double desiredChange = targetPowerBlue - currentPowerBlue;
//        double limitedChange = Math.max(-SLEW_RATE_BLUE, Math.min(desiredChange, SLEW_RATE_BLUE));
//        launcherMotorBlue.setPower(currentPowerBlue += limitedChange);
//
//        this.m_launcherMotorRed.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.m_launcherMotorBlue.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
    //DcMotor launcherMotorRed, double targetPowerRed, DcMotor launcherMotorBlue, double targetPowerBlue
    public void launch ()
    {
//        setSafePower(launcherMotorRed, targetPowerRed, launcherMotorBlue, targetPowerBlue);
        this.m_launcherMotorRed.setPower(1.0);
//        this.m_launcherMotorBlue.setPower(0.8);

        this.m_launcherMotorRed.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.m_launcherMotorBlue.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop()
    {
        this.m_launcherMotorRed.setPower(0);
        this.m_launcherMotorRed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.m_launcherMotorBlue.setPower(0);
//        this.m_launcherMotorBlue.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
