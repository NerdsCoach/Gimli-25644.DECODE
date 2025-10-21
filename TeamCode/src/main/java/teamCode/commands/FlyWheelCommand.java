//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import teamCode.subsystems.FlyWheelSubsystem;
//
//public class FlyWheelCommand extends CommandBase
//{
//    private FlyWheelSubsystem m_flyWheelSubsystem;
//
//    public FlyWheelCommand(FlyWheelSubsystem flyWheelSubsystem)
//    {
//        this.m_flyWheelSubsystem = flyWheelSubsystem;
//
//        addRequirements(m_flyWheelSubsystem);
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
//        this.m_flyWheelSubsystem.spin();
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
