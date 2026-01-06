package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.LauncherSubsystem;

public class LauncherCommand extends CommandBase
{
    private LauncherSubsystem m_launcherSubsystem;

    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private final AxeSubsystem m_axeSubsystem;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;

    public LauncherCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem)
    {
        m_position = m_down;

        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        addRequirements(m_launcherSubsystem, this.m_axeSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        if (m_position == m_down)
        {
            this.m_axeSubsystem.pivotAxe(m_axeUp);
            this.m_launcherSubsystem.launch();
            m_position = m_up;
        }
        else if (m_position == m_up)
        {
            this.m_axeSubsystem.pivotAxe(m_axeDown);
            this.m_launcherSubsystem.stop();
            m_position = m_down;
        }
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
