//package teamCode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//
//import teamCode.DriveToPoint;
//import teamCode.GoBildaPinpointDriver;
//import teamCode.Pose2DUnNormalized;
//
//public class PinPointOdometrySubsystem extends SubsystemBase
//{
//    private GoBildaPinpointDriver m_odo;
//    DriveToPoint nav = new DriveToPoint();
//    private DcMotor m_leftFront;
//    private DcMotor m_rightFront;
//    private DcMotor m_leftBack;
//    private DcMotor m_rightBack;
////    private double oldTime = 0;
////    private Pose2DUnNormalized testPose = new Pose2DUnNormalized(DistanceUnit.MM, 100, 100, UnnormalizedAngleUnit.DEGREES, 0.0);
//
//
//    public PinPointOdometrySubsystem(teamCode.GoBildaPinpointDriver odo, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack)
//    {
//        this.m_odo = odo;
//        this.m_odo.setOffsets(68,-178);
//        this.m_odo.setEncoderResolution(teamCode.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        this.m_odo.setEncoderDirections(teamCode.GoBildaPinpointDriver.EncoderDirection.REVERSED, teamCode.GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.m_odo.setPosition(new Pose2DUnNormalized(DistanceUnit.MM, 0,0, UnnormalizedAngleUnit.DEGREES, 0.0));
//        this.m_leftFront = leftFront;
//        this.m_rightFront = rightFront;
//        this.m_leftBack = leftBack;
//        this.m_rightBack = rightBack;
//    }
//
//    public void autoSetPower()
//    {
//        //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
//
//        this.m_leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
//        this.m_rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
//        this.m_leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
//        this.m_rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
//        System.out.println("Power"+nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
//    }
//
//
//
//
////    public class ArmSubsystem {
////        // Method that takes a DriveSubsystem object
////        public void orientArm(DriveSubsystem drive) {
////            double robotHeading = drive.getHeading();
//
//
//
//    public void resetOdo()
//    {
//        this.m_odo.resetPosAndIMU();
//    }
//
//    public Pose2DUnNormalized getUnNormalizedPosition()
//    {
//        return this.m_odo.getUnNormalizedPosition();
//    }
//
//    public void updateOdo()
//    {
//        this.m_odo.update();
//    }
//
//    public double[] getDeltaPosition(double target)
//    {
//        return new double[]
//                {
//                        (target - m_odo.getPosX()) / target,
//                        (target - m_odo.getPosY()) / target
//                };
//    }
//
//}