package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.ParkingSubsystem;

public class DeParkingCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;

    public DeParkingCommand(ParkingSubsystem parkingSubsystem)
    {
        this.m_parkingSubsystem = parkingSubsystem;

        addRequirements(m_parkingSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
            this.m_parkingSubsystem.parking(Constants.ParkConstants.kDeParkLimit);
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
