//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import teamCode.subsystems.LightSubsystem;
//
//public class ZLightCommand extends CommandBase
//{
//    private final LightSubsystem m_lightSubsystem;
//
//    public ZLightCommand(LightSubsystem light)
//    {
//        this.m_lightSubsystem = light;
//        addRequirements(this.m_lightSubsystem);
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
//        this.m_lightSubsystem.on(0.722);
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return true;
//    }
//
//    @Override
//    public void end(boolean interrupted)
//    {
//    }
//}
