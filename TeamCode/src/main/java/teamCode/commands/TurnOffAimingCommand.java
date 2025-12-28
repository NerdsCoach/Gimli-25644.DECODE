package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class TurnOffAimingCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.01; // Proportional gain (needs tuning)

    public TurnOffAimingCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem)
    {
        m_huskySubsystem = huskyLensSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
        addRequirements(huskyLensSubsystem, turnTableSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        m_turnTableSubsystem.stop(); // Stop if no tag detected
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
    }
}