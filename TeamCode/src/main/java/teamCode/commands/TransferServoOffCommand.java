package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.TransferSubsystem;

public class TransferServoOffCommand extends CommandBase
{
    private final TransferSubsystem m_transferServoSubsystem;

    public TransferServoOffCommand(TransferSubsystem transfer)
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
        this.m_transferServoSubsystem.spinTransfer(0.0);
    }
}
