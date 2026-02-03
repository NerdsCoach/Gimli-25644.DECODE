package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.ParkingSubsystem;

public class DefenseUpCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;

    public DefenseUpCommand(ParkingSubsystem parkingSubsystem)
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
            this.m_parkingSubsystem.parking(Constants.ParkConstants.kDefenseUp);
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
