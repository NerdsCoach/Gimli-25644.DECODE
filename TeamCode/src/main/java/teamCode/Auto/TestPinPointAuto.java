//package teamCode.Auto;
//
//import static teamCode.Constants.LiftArmConstants.kLiftArmIntakeReset;
//import static teamCode.PoseStorage.odoHeading;
//import static teamCode.PoseStorage.xEncoder;
//import static teamCode.PoseStorage.yEncoder;
//import static teamCode.PoseStorage.yReading;
//
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import teamCode.DriveToPoint;
//
//import java.util.Locale;
//
//import teamCode.Pose2DUnNormalized;
//import teamCode.PoseStorage;
//import teamCode.commands.ArmIntakeResetCommand;
//import teamCode.commands.ArmPositionCloseSampleCommand;
//import teamCode.commands.ArmPositionHighBasketCommand;
//import teamCode.commands.ArmPositionHighChamberCommand;
//import teamCode.commands.ArmPositionTravelCommand;
//import teamCode.commands.IntakePivotCommand;
//import teamCode.commands.SlideFudgeInCommand;
//import teamCode.commands.StingrayAscent1ArmCommand;
//import teamCode.subsystems.PurplePotatoSorterSubsystemOG;
//import teamCode.subsystems.IntakeServoSubsystem;
//import teamCode.subsystems.LiftArmSubsystem;
//import teamCode.subsystems.BilbosTurnTableSubsystem;
//import teamCode.subsystems.StingRayArmSubsystem;
//
//@Autonomous(name="TestPinPointAuto", group="Pinpoint")
////@Disabled
//
//public class TestPinPointAuto extends LinearOpMode
//{
//
//    private DcMotor leftFront;
//    private DcMotor rightFront;
//    private DcMotor leftBack;
//    private DcMotor rightBack;
//    private DcMotor m_liftArmMotor;
//    private DcMotor m_slideArmMotor;
//    private CRServo m_intakeWheelServo;
//    private PurplePotatoSorterSubsystemOG m_intakePivotSubsystem;
//    private StingRayArmSubsystem m_ascentArmSubsystem;
//    private LiftArmSubsystem m_liftArmSubsystem;
//    private BilbosTurnTableSubsystem m_slideArmSubsystem;
//    private IntakeServoSubsystem m_intakeWheelSubsystem;
//    private ArmPositionHighBasketCommand m_armPositionHighBasketCommand;
//    private ArmPositionHighChamberCommand m_armPositionHighChamberCommand;
//    private ArmPositionTravelCommand m_armPositionHomeCommand;
//    private ArmPositionCloseSampleCommand m_armPositionCloseSampleCommand;
//    private ArmIntakeResetCommand m_armIntakeResetCommand;
//    private IntakePivotCommand m_intakePivotCommand;
//    private StingrayAscent1ArmCommand m_ascentArmCommand;
//    private SlideFudgeInCommand m_slideFudgeInCommand;
//    private TouchSensor m_touch;
//    private final ElapsedTime holdTimer = new ElapsedTime();
//
//    GoBildaPinpointDriver m_odo; // Declare OpMode member for the Odometry Computer
////    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
//    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class
//
//
//    enum StateMachine
//    {
//        WAITING_FOR_START,
//        SCORE_PRELOAD,
//        TURN_OFF_INTAKE_1,
//        DRIVE_TO_SAMPLE_2,
//        PICKUP_SAMPLE_2,
//        SCORE_SAMPLE_2,
//        TURN_OFF_INTAKE_2,
//        DRIVE_TO_SAMPLE_3,
//        PICKUP_SAMPLE_3,
//        SCORE_SAMPLE_3,
//        TURN_OFF_INTAKE_3,
//        DRIVE_TO_SAMPLE_4,
//        PICKUP_SAMPLE_4,
//        SCORE_SAMPLE_4,
//        TURN_OFF_INTAKE_4,
//        PRE_PARK,
//        PARK_ASCENT_1,
//        PARKED,
//    }
//
////    static final Pose2DUnNormalized NET_ZONE = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, -45);
////    static final Pose2DUnNormalized PrePickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 0, 0, UnnormalizedAngleUnit.DEGREES, 0);
////    static final Pose2DUnNormalized PickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, 45);
////    static final Pose2DUnNormalized PrePickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 0,0 , UnnormalizedAngleUnit.DEGREES, 180);
////    static final Pose2DUnNormalized PickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, -180);
////
////    static final Pose2DUnNormalized PrePickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 560, 640, UnnormalizedAngleUnit.DEGREES, 44);
////    static final Pose2DUnNormalized PickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 820, 690, UnnormalizedAngleUnit.DEGREES, 44);
////    static final Pose2DUnNormalized PrePark = new Pose2DUnNormalized(DistanceUnit.MM, 1165, 390, UnnormalizedAngleUnit.DEGREES, 90);
////    static final Pose2DUnNormalized ParkAscent1 = new Pose2DUnNormalized(DistanceUnit.MM, 1270, -220, UnnormalizedAngleUnit.DEGREES, 90);
//
//    static final Pose2DUnNormalized NET_ZONE = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, -45);
//    static final Pose2DUnNormalized PrePickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 0, 0, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, 45);
//    static final Pose2DUnNormalized PrePickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 0,0 , UnnormalizedAngleUnit.DEGREES, 180);
//    static final Pose2DUnNormalized PickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 200, 200, UnnormalizedAngleUnit.DEGREES, -180);
//
//    static final Pose2DUnNormalized PrePickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 560, 640, UnnormalizedAngleUnit.DEGREES, 44);
//    static final Pose2DUnNormalized PickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 820, 690, UnnormalizedAngleUnit.DEGREES, 44);
//    static final Pose2DUnNormalized PrePark = new Pose2DUnNormalized(DistanceUnit.MM, 1165, 390, UnnormalizedAngleUnit.DEGREES, 90);
//    static final Pose2DUnNormalized ParkAscent1 = new Pose2DUnNormalized(DistanceUnit.MM, 1270, -220, UnnormalizedAngleUnit.DEGREES, 90);
//
//    @Override
//    public void runOpMode()
//    {
//
//        // Initialize the hardware variables. Note that the strings used here must correspond
//        // to the names assigned during the robot configuration step on the DS or RC devices.
//
//        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
//        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        m_odo = hardwareMap.get(GoBildaPinpointDriver.class,"m_odo");
////        m_odo.setOffsets(68, -178);//these are tuned for Sting-Ray 3110-0002-0001 Product Insight #1
////        m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
////        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
////        m_odo.resetPosAndIMU();
//        this.m_odo.setPosition(PoseStorage.zero);
//
//        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
//        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
//        nav.setDriveType(DriveToPoint.DriveType.MECANUM);
//
//        StateMachine stateMachine;
//        stateMachine = StateMachine.WAITING_FOR_START;
//
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", m_odo.getXOffset(DistanceUnit.MM));
//        telemetry.addData("Y offset", m_odo.getYOffset(DistanceUnit.MM));
//        telemetry.addData("Device Version Number:", m_odo.getDeviceVersion());
//        telemetry.addData("Device Scalar", m_odo.getYawScalar());
//        telemetry.update();
//
//        this.m_liftArmMotor = hardwareMap.get(DcMotor.class, "liftArmMotor");
//        this.m_slideArmMotor = hardwareMap.get(DcMotor.class, "slideArmMotor");
//
//        this.m_intakeWheelServo = new CRServo(hardwareMap, "intakeWheelServo");
//        this.m_intakePivotSubsystem = new PurplePotatoSorterSubsystemOG(hardwareMap, "intakePivotServo");
//        this.m_ascentArmSubsystem = new StingRayArmSubsystem(hardwareMap, "ascentArmServo");
//
//        this.m_liftArmSubsystem = new LiftArmSubsystem(this.m_liftArmMotor);
//        this.m_slideArmSubsystem = new BilbosTurnTableSubsystem(this.m_slideArmMotor);
//        this.m_intakeWheelSubsystem = new IntakeServoSubsystem(this.m_intakeWheelServo);
//
//        this.m_armPositionHomeCommand = new ArmPositionTravelCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem, this.m_intakePivotSubsystem);
//        this.m_armPositionCloseSampleCommand = new ArmPositionCloseSampleCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem, this.m_intakePivotSubsystem);
//        this.m_armPositionHighBasketCommand = new ArmPositionHighBasketCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem, this.m_intakePivotSubsystem);
//        this.m_armPositionHighChamberCommand = new ArmPositionHighChamberCommand(this.m_liftArmSubsystem,this.m_slideArmSubsystem, this.m_intakePivotSubsystem);
//        this.m_intakePivotCommand = new IntakePivotCommand(this.m_intakePivotSubsystem);
//        this.m_ascentArmCommand = new StingrayAscent1ArmCommand(this.m_ascentArmSubsystem);
//        //this.m_armFudgeFactorUpCommand = new ArmFudgeFactorUpCommand(this.m_liftArmSubsystem);
//
//        waitForStart();
//        resetRuntime();
//
//        while (opModeIsActive())
//        {
//            m_odo.update();
//
//            switch (stateMachine)
//            {
//                case WAITING_FOR_START:
//                    this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                    //the first step in the autonomous
//                    stateMachine = StateMachine.SCORE_PRELOAD;
//                    break;
//
//
//                case SCORE_PRELOAD:
//                    /*
//                    drive the robot to the first target, the nav.driveTo function will return true once
//                    the robot has reached the target, and has been there for (holdTime) seconds.
//                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
//                     */
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.5, 0))
//                    {
//                        telemetry.addLine("Score Sample 1");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_2;
//                    }
//                    break;
//
//
//
//                case DRIVE_TO_SAMPLE_2:  //drive to the Sample 2
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PrePickUpSample2, 0.4, 0))
//                    {
//                        telemetry.addLine("Ready to pick up Sample 2!");
//                        stateMachine = StateMachine.PICKUP_SAMPLE_2;
//                    }
//                    break;
//
//
//                case PICKUP_SAMPLE_2:
//                    telemetry.addLine("Yes!");
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample2, 0.3, 0))
//                    {
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_2;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_2:
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.4,0))
//                    {
//                        telemetry.addLine("Ready to drive 3");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_3;
//                    }
//                    break;
//
//
//                case DRIVE_TO_SAMPLE_3:  //drive to the Sample 2
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PrePickUpSample3, 0.4, .5))
//                    {
//                    telemetry.addLine("Ready to pick up Sample 3!");
//                    stateMachine = StateMachine.PICKUP_SAMPLE_3;
//                    }
//                    break;
//
//
//                case PICKUP_SAMPLE_3:
//
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample3, 0.3, 0))
//                    {
//                        stateMachine = StateMachine.SCORE_SAMPLE_3;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_3:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.4, 9.0))
//                    {
//                        telemetry.addLine("Score Sample 3");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_3;
////                        posi = this.m_odo.getPositionDegrees();
//
//                    }
//                    break;
//
//
//                }
//
//
//            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
//            leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
//            rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
//            leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
//            rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
//
//            telemetry.addData("current state:",stateMachine);
//
//            Pose2DUnNormalized pos = new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES));
//
////            currentPose = m_odo.getUnNormalizedPosition();
//            xEncoder = this.m_odo.getPosX(DistanceUnit.MM);
//            yEncoder = this.m_odo.getPosY(DistanceUnit.MM);
//            odoHeading = this.m_odo.getHeading(UnnormalizedAngleUnit.DEGREES);
//
//
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
//            telemetry.addData("Position", data);
////            telemetry.addData("PoseStorage",posi);
//            telemetry.addData("X",xEncoder);
//            telemetry.addData("Y",yReading);
//            telemetry.addData("Heading",odoHeading);
//            telemetry.update();
//
//        }
//    }
//}   // end class
//
