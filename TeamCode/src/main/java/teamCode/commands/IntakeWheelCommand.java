//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import java.util.function.DoubleSupplier;
//import teamCode.subsystems.SecondBreakfastIntakeSubsystem;
//
//public class IntakeWheelCommand extends CommandBase
//{
//    private final SecondBreakfastIntakeSubsystem m_intakeWheelSubsystem;
//    private DoubleSupplier m_rightTriggerValue;
//    private DoubleSupplier m_leftTriggerValue;
//
//    public IntakeWheelCommand(SecondBreakfastIntakeSubsystem wheel, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger)
//    {
//        this.m_intakeWheelSubsystem = wheel;
//        this.m_rightTriggerValue = rightTrigger;
//        this.m_leftTriggerValue = leftTrigger;
//        addRequirements(this.m_intakeWheelSubsystem);
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
//        if (this.m_intakeWheelSubsystem.isLoaded() && this.m_rightTriggerValue.getAsDouble() > 0.1)
//        {
//            this.m_intakeWheelSubsystem.spinIntake(0.0);
//        }
//        else
//        {
//            this.m_intakeWheelSubsystem.spinIntake(
//                this.m_rightTriggerValue.getAsDouble() * -1 - this.m_leftTriggerValue.getAsDouble() * -1);//changed after 7 rivers qualifier was .5
//        }
//   }
//}
