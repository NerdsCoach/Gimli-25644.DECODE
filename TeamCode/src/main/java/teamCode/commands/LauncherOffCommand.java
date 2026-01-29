package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.TreeMap;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LauncherSubsystem;

public class LauncherOffCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private final HuskyLensSubsystem m_huskySubsystem;

    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_down = 0;

    public LauncherOffCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem, HuskyLensSubsystem huskyLensSubsystem)
    {
        m_position = m_down;

        this.m_launcherSubsystem = launcherSubsystem;
        m_huskySubsystem = huskyLensSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
            this.m_axeSubsystem.pivotAxe(m_axeUp);
            this.m_launcherSubsystem.setMotorVelocity(0);
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
