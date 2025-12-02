package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.GimlisBoxMotorSubsystem;
import teamCode.Constants;

public class ParkingCommand extends CommandBase
{
    private GimlisBoxMotorSubsystem m_parkingSubsystem;
    private DoubleSupplier m_rightTriggerValue;
    private DoubleSupplier m_leftTriggerValue;

    public ParkingCommand(GimlisBoxMotorSubsystem parkingSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger)
    {
        this.m_parkingSubsystem = parkingSubsystem;
        this.m_rightTriggerValue = rightTrigger;
        this.m_leftTriggerValue = leftTrigger;

        addRequirements(m_parkingSubsystem);
    }

        @Override
        public void initialize()
        {

        }

        @Override
        public void execute()
        {
            this.m_parkingSubsystem.spinIntake(
                    this.m_rightTriggerValue.getAsDouble() * -1 - this.m_leftTriggerValue.getAsDouble() * -0.5);
        }

//
//    @Override
//    public void execute()
//    {
//       this.m_parkingSubsystem.Park(Constants.ParkConstants.kLiftPark);
//    }
//
//
//    @Override
//    public void initialize()
//    {
//    }
//
//    @Override
//    public void end(boolean interrupted)
//    {
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return true;
//    }
}
