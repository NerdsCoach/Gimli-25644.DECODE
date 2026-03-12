package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;

public class AxeToggleCommand extends CommandBase
    {
        private final AxeSubsystem m_axeSubsystem;
        private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
        private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

        public AxeToggleCommand(AxeSubsystem axeSubsystem)
        {
            this.m_axeSubsystem = axeSubsystem;

            addRequirements(this.m_axeSubsystem);
        }

        @Override
        public void execute()
        {
            this.m_axeSubsystem.pivotAxe(m_axeUp);

        }

        @Override
        public void end(boolean interrupted)
        {
            this.m_axeSubsystem.pivotAxe(m_axeDown);
        }

        @Override
        public boolean isFinished()
        { return false; }
    }
