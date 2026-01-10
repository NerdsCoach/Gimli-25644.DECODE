package teamCode.commands;
import teamCode.Constants;
import teamCode.subsystems.TurnTableSubsystem;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class TurnTableLeftCommand extends CommandBase
{
    /* Uses triggers to manually adjust turnSpeed table */
    private TurnTableSubsystem m_turnTableSubsystem;

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
//            this.m_turnTableSubsystem.turnTable(
//                    this.m_rightTriggerValue.getAsDouble() * -1 - this.m_leftTriggerValue.getAsDouble() * -1);
//            //If trigger directions are backwards switch right and left

            if (!m_turnTableSubsystem.atTargetLeft(850))
            {
                this.m_turnTableSubsystem.newTurnTable(Constants.TurnTableConstants.kTurnTableLeft);
            }
        }
    }