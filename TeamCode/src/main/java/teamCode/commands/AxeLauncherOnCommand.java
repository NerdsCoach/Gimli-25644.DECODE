package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.TransferSubsystem;

public class AxeLauncherOnCommand extends CommandBase
{
    private final AxeSubsystem m_axeSubsystem;
    private final LauncherSubsystem m_launcherSubsystem;
    private final TransferSubsystem m_transferSubsystem;
    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_Up = 0;
    private static final int m_Down = 1;

    public AxeLauncherOnCommand(AxeSubsystem axeSubsystem, LauncherSubsystem launcherSubsystem, TransferSubsystem transferSubsystem)
    {
        this.m_axeSubsystem = axeSubsystem;
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_transferSubsystem = transferSubsystem;
        m_position = m_Up;
        addRequirements(this.m_axeSubsystem, this.m_launcherSubsystem, this.m_transferSubsystem);
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {

        /* If Axe is up, put it down and turn on Launcher Motor
        *   if launcher motor is running, turn on transfer */
        if (m_position == m_Up)
        {
            this.m_axeSubsystem.pivotAxe(m_axeDown);
            this.m_launcherSubsystem.launch();
            if (this.m_launcherSubsystem.atTarget())
            {
                this.m_transferSubsystem.spinTransfer(1);
                m_position = m_Down;
            }

        }

        //TODO If limit switch == 3 (Maybe a state machine?) stop transfer and Launcher and Lift axe,
        // otherwise keep going
        else if (m_position == m_Down)
        {
            this.m_axeSubsystem.pivotAxe(m_axeUp);
            m_position = m_Up;
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
