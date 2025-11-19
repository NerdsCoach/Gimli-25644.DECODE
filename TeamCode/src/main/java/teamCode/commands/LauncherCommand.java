package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import teamCode.Constants;
import teamCode.subsystems.LauncherMotorSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class LauncherCommand extends CommandBase
{
    private LauncherMotorSubsystem m_launcherSubsystem;
    DcMotor m_launcherMotorRed;
    DcMotor m_launcherMotorBlue;

    public LauncherCommand(LauncherMotorSubsystem launcherSubsystem)
    {
        this.m_launcherSubsystem = launcherSubsystem;


        addRequirements(m_launcherSubsystem);
    }

    @Override
    public void initialize()
    {
    }
//    DcMotor launcherMotorRed, double targetPowerRed, DcMotor launcherMotorBlue, double targetPowerBlue
//    launcherMotorRed, targetPowerRed, launcherMotorBlue, targetPowerBlue

    @Override
    public void execute()
    {
        this.m_launcherSubsystem.launch();
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
