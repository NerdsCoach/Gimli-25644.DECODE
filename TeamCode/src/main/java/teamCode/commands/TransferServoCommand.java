package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TransferServoSubsystem;

public class TransferServoCommand extends CommandBase
{
    private final TransferServoSubsystem m_transferServoSubsystem;

    public TransferServoCommand(TransferServoSubsystem transfer)
    {
        this.m_transferServoSubsystem = transfer;
        addRequirements(this.m_transferServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_transferServoSubsystem.spinTransfer(-.75);
    }
}
