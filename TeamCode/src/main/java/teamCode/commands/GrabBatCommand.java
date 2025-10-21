//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import teamCode.subsystems.GrabBatSubsystem;
//
//public class GrabBatCommand extends CommandBase
//{
//    private static final double m_closedPos = 0.8;
//    private static final double m_openPos = 0.3;
//    private final GrabBatSubsystem m_grabBatSubsystem;
//    private int m_position;
//    private static final int m_open = 0;
//    private static final int m_closed = 1;
//
//    public GrabBatCommand(GrabBatSubsystem pivotSubsystem)
//    {
//        this.m_grabBatSubsystem = pivotSubsystem;
//        m_position = m_closed;
//        addRequirements(this.m_grabBatSubsystem);
//    }
//
//    @Override
//    public void initialize()
//    {
//    }
//    @Override
//    public void execute()
//    {
//        if (m_position == m_open)
//        {
//            this.m_grabBatSubsystem.pivotGrabBatServo(m_closedPos);
//            m_position = m_closed;
//        }
//        else if (m_position == m_closed)
//        {
//            this.m_grabBatSubsystem.pivotGrabBatServo(m_openPos);
//            m_position = m_open;
//        }
//
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