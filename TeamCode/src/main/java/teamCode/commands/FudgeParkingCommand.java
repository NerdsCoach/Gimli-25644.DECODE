package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.ParkingSubsystem;

public class FudgeParkingCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;

public FudgeParkingCommand(ParkingSubsystem parkingSubsystem)
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
            if (!m_parkingSubsystem.atTargetUp(Constants.ParkConstants.kParkLiftLimit))
            {
                this.m_parkingSubsystem.parkFudgeFactor(Constants.ParkConstants.kLiftPark);

            }
        }

}
