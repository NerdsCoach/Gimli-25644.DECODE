package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.subsystems.TurnTableSubsystem;

public class ResetTurnTableCommand extends CommandBase
{
    private final TurnTableSubsystem m_turnTableSubsystem;

        public ResetTurnTableCommand(TurnTableSubsystem turnTableSubsystem)
        {
            this.m_turnTableSubsystem = turnTableSubsystem;
            addRequirements(m_turnTableSubsystem);
        }
    @Override
    public void initialize()
        {
            m_turnTableSubsystem.Turn(0);
        }

    @Override
    public void execute()
        {
        }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.setAimingMode();
        m_turnTableSubsystem.stop();
    }

    @Override
    public boolean isFinished()
    {
        return m_turnTableSubsystem.isAtTarget();

//        return false;
    }

}