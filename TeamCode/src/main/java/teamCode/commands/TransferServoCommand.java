package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import static teamCode.Constants.TransferConstants.kTransferDown;
import static teamCode.Constants.TransferConstants.kTransferUp;
import teamCode.Constants;
import teamCode.subsystems.TransferServoSubsystem;

public class TransferServoCommand extends CommandBase
{
        private static final double m_transferPos1 = kTransferDown;
        private static final double m_transferPos2 = kTransferUp;
        private final TransferServoSubsystem m_transferServoSubsystem;
        private double m_position;
        private static final double  m_pos1= 1;
        private static final int m_pos2 = 2;
        private static final int m_pos3 = 3;

    public TransferServoCommand(TransferServoSubsystem transferSubsystem)
        {
            this.m_transferServoSubsystem = transferSubsystem;
            m_position = m_pos1;
            addRequirements(this.m_transferServoSubsystem);
        }

        @Override
        public void initialize()
        {
        }

        @Override
        public void execute()
        {
            if (m_position == m_pos1)
            {
                this.m_transferServoSubsystem.transfer(m_transferPos2);
                m_position = m_pos2;
            }
            else if (m_position == m_pos2)
            {
                this.m_transferServoSubsystem.transfer(m_transferPos1);
                m_position = m_pos1;
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
