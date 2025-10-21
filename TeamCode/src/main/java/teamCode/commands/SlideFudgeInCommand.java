//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import teamCode.Constants;
//import teamCode.subsystems.BilbosTurnTableSubsystem;
//
//public class SlideFudgeInCommand extends CommandBase
//{
//    private BilbosTurnTableSubsystem m_slideArmSubsystem;
//
//    public SlideFudgeInCommand(BilbosTurnTableSubsystem slideArmSubsystem)
//    {
//        this.m_slideArmSubsystem = slideArmSubsystem;
//
//        addRequirements(m_slideArmSubsystem);
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
//        this.m_slideArmSubsystem.slideFudgeFactor(Constants.SlideArmConstants.kSlideFudgeIn);
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
