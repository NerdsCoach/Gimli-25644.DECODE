package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.subsystems.LimeLightSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingOffCommand extends CommandBase
{
    private final LimeLightSubsystem m_limeLightSubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;

    public AimingOffCommand(LimeLightSubsystem limeLightSubsystem, TurnTableSubsystem turnTableSubsystem)
    {
        m_limeLightSubsystem = limeLightSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
        addRequirements(turnTableSubsystem);
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