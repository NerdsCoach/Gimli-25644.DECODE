package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.Constants;
import teamCode.subsystems.GimlisBoxMotorSubsystem;

public class ParkingCommand extends CommandBase
{
    private GimlisBoxMotorSubsystem m_parkingSubsystem;

    public ParkingCommand(GimlisBoxMotorSubsystem parkingSubsystem)
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
        this.m_parkingSubsystem.liftPark(Constants.ParkConstants.kLiftPark);
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
