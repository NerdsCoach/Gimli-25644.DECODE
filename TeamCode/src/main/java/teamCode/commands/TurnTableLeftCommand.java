package teamCode.commands;
import teamCode.Constants;
import teamCode.subsystems.TurnTableSubsystem;
import com.arcrobotics.ftclib.command.CommandBase;


public class TurnTableLeftCommand extends CommandBase
{
    /* Uses triggers to manually adjust turnSpeed table */
    private final TurnTableSubsystem m_turnTableSubsystem;

        public TurnTableLeftCommand(TurnTableSubsystem turnTableSubsystem)
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
            if (!m_turnTableSubsystem.atTargetLeft(-835))
            {
                this.m_turnTableSubsystem.newTurnTable(Constants.TurnTableConstants.kTurnTableLeft);
            }
        }
    }