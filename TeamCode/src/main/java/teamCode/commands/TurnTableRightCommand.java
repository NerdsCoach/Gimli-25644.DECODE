package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.Constants;
import teamCode.subsystems.TurnTableSubsystem;

public class TurnTableRightCommand extends CommandBase
{
    /* Uses triggers to manually adjust turn table */
    private TurnTableSubsystem m_turnTableSubsystem;

        public TurnTableRightCommand(TurnTableSubsystem turnTableSubsystem)
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
            if (!m_turnTableSubsystem.atTargetRight(-850))
            {
                this.m_turnTableSubsystem.newTurnTable(Constants.TurnTableConstants.kTurnTableRight);
            }
        }
    }