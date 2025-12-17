//package teamCode.commands;
//
//import static teamCode.Constants.SorterConstants.kSorterPos1;
//import static teamCode.Constants.SorterConstants.kSorterPos2;
//import static teamCode.Constants.SorterConstants.kSorterPos3;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import teamCode.subsystems.SorterServoSubsystem;
//
//public class IntakeResetCommand extends CommandBase
//{
//    private SorterServoSubsystem m_sorterServoSubsystem;
//
//    private static final double m_sorterPos1 = kSorterPos1;
//    private double m_position;
//    private static final double  m_pos1= 1;
//
//    public IntakeResetCommand(SorterServoSubsystem sorterSubsystem)
//    {
//        this.m_sorterServoSubsystem = sorterSubsystem;
//        m_position = m_pos1;
//        addRequirements(m_sorterServoSubsystem);
//    }
//
//    @Override
//    public void initialize()
//    {
//    }
//
//    @Override
//    public void execute()
//    {
////        this.m_sorterServoSubsystem.sort(kSorterPos1);
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
//}
