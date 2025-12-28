//package teamCode.commands;
//import teamCode.Constants;
//import teamCode.subsystems.TurnTableSubsystem;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//public class TurnTableCommand extends CommandBase
//{
//    private TurnTableSubsystem m_turnTableSubsystem;
//
//        public TurnTableCommand(TurnTableSubsystem turnTableSubsystem)
//        {
//            this.m_turnTableSubsystem = turnTableSubsystem;
//
//            addRequirements(m_turnTableSubsystem);
//        }
//
//        @Override
//        public void initialize()
//        {
//        }
//
//        @Override
//        public void execute()
//        {
////            this.m_turnTableSubsystem.turnTable(85);
////            this.m_turnTableSubsystem.fudgeFactor();
////                this.m_turnTableSubsystem.turnTable(Constants.TurnTableConstants.kTurnTable);
////            this.m_turnTableSubsystem.turnTable(Constants.TurnTableConstants.kTurnTable);
//            this.m_turnTableSubsystem.turnTable();
//        }
//
//        @Override
//        public void end(boolean interrupted)
//        {
//        }
//
//        @Override
//        public boolean isFinished()
//        {
//            return true;
//        }
//    }
//
