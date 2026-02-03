package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.TurnTableSubsystem;

public class ResetTurnTableCommand extends CommandBase
{
    /* Uses triggers to manually adjust turnSpeed table */
    private final TurnTableSubsystem m_turnTableSubsystem;

        public ResetTurnTableCommand(TurnTableSubsystem turnTableSubsystem)
        {
            this.m_turnTableSubsystem = turnTableSubsystem;
            addRequirements(m_turnTableSubsystem);
        }

        @Override
        public void initialize()
        {
        }

        @Override
        public void execute()
        {
            m_turnTableSubsystem.Turn(0);
            //TODO add to red and blue robot containers
        }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    } //try return false

}