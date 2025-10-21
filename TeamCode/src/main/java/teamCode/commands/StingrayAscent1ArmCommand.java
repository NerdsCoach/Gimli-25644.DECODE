//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import teamCode.subsystems.StingRayArmSubsystem;
//
//public class StingrayAscent1ArmCommand extends CommandBase
//{
//    private static final double m_homePos = 0.0;
//    private static final double m_scorePos = 0.7;
//    private final StingRayArmSubsystem m_ascentArmSubsystem;
//    private int m_position;
//    private static final int  m_home = 1;
//    private static final int  m_score = 0;
//
//    public StingrayAscent1ArmCommand(StingRayArmSubsystem ascentSubsystem)
//    {
//        this.m_ascentArmSubsystem = ascentSubsystem;
//        m_position = m_score;
//        addRequirements(this.m_ascentArmSubsystem);
//    }
//
//    @Override
//    public void initialize()
//    {
//    }
//    @Override
//    public void execute()
//    {
//        if (m_position == m_score)
//        {
//            this.m_ascentArmSubsystem.ascentArm(m_scorePos);
//            m_position = m_home;
//        }
//        else if (m_position == m_home)
//        {
//            this.m_ascentArmSubsystem.ascentArm(m_homePos);
//            m_position = m_score;
//        }
//
//    }
//
//    public void autoAscent()
//    {
//        this.m_ascentArmSubsystem.ascentArm(m_scorePos);
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