//package teamCode.subsystems;
//
//import static teamCode.PoseStorage.currentPose;
//import static teamCode.PoseStorage.odoHeading;
//import static teamCode.PoseStorage.xEncoder;
//import static teamCode.PoseStorage.yEncoder;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//
//import teamCode.DriveToPoint;
////import teamCode.GoBildaPinpointDriver;
//import teamCode.GoBildaPinpointReader;
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
//    public PinPointOdometrySubsystem(GoBildaPinpointDriver odo, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack)
//    {
//        this.m_odo = odo;
//        this.m_odo = new GoBildaPinpointDriver();
////        this.m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        this.m_odo.setOffsets(87, -170, DistanceUnit.MM);//68,-178 are for Sting-Ray 3110-0002-0001 Product Insight #1
//        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.m_odo.setPosX(xEncoder,DistanceUnit.MM);
//        this.m_odo.setPosY(yEncoder,DistanceUnit.MM);
////        this.m_odo.setHeading(odoHeading,UnnormalizedAngleUnit.DEGREES);
//        this.m_odo.setHeading(odoHeading, AngleUnit.DEGREES);
//        double headingDegrees = currentPose.getHeading(UnnormalizedAngleUnit.DEGREES);
////        this.m_odo.setOffsets(68,-178);
////        this.m_odo.setEncoderResolution(teamCode.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
////        this.m_odo.setEncoderDirections(teamCode.GoBildaPinpointDriver.EncoderDirection.REVERSED, teamCode.GoBildaPinpointDriver.EncoderDirection.FORWARD);
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