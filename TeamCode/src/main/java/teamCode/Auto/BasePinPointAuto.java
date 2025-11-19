//package teamCode.Auto;
//
//import static teamCode.Constants.LiftArmConstants.kLiftArmCloseSample;
//import static teamCode.Constants.LiftArmConstants.kLiftArmHighBasket;
//import static teamCode.Constants.LiftArmConstants.kLiftArmIntakeReset;
//import static teamCode.Constants.SorterConstants.kIntakePivotPickUp;
//import static teamCode.Constants.SorterConstants.kIntakePivotScore;
//import static teamCode.Constants.SlideArmConstants.kSlideArmCloseSample;
//import static teamCode.Constants.SlideArmConstants.kSlideArmHighBasket;
//
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import teamCode.DriveToPoint;
//import org.firstinspires.ftc.teamcode.Libs.GoBilda.GoBildaPinpointDriver;
//
//import java.util.Locale;
//
//import teamCode.autoSubsystems.AutoDriveSubsystem;
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
//@Autonomous(name="BasePinPointAuto", group="Pinpoint")
////@Disabled
//
//public class BasePinPointAuto extends LinearOpMode
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
//    private AutoDriveSubsystem m_autoDriveSubsystem;
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
//    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
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
//    static final Pose2DUnNormalized NET_ZONE = new Pose2DUnNormalized(DistanceUnit.MM, 250, 470, UnnormalizedAngleUnit.DEGREES, -45);
//    static final Pose2DUnNormalized PrePickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 400, 420, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 640, 420, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PrePickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 400, 675, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 680, 675, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PrePickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 560, 640, UnnormalizedAngleUnit.DEGREES, 44);
//    static final Pose2DUnNormalized PickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 820, 690, UnnormalizedAngleUnit.DEGREES, 44);
//    static final Pose2DUnNormalized PrePark = new Pose2DUnNormalized(DistanceUnit.MM, 1165, 390, UnnormalizedAngleUnit.DEGREES, 90);
//    static final Pose2DUnNormalized ParkAscent1 = new Pose2DUnNormalized(DistanceUnit.MM, 1270, -220, UnnormalizedAngleUnit.DEGREES, 90);
//
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
//        m_odo.setOffsets(68, -178);//these are tuned for Sting-Ray 3110-0002-0001 Product Insight #1
//        m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//        m_odo.resetPosAndIMU();
//
//        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
//        //nav.setYawCoefficients(1,0,0.0, UnnormalizedAngleUnit.DEGREES,2);
//        nav.setDriveType(DriveToPoint.DriveType.MECANUM);
//
//        StateMachine stateMachine;
//        stateMachine = StateMachine.WAITING_FOR_START;
//
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", m_odo.getXOffset());
//        telemetry.addData("Y offset", m_odo.getYOffset());
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
//        this.m_intakeWheelSubsystem = new IntakeServoSubsystem(this.m_intakeWheelServo,this.m_touch);
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
//                    if (nav.driveTo(m_odo.getUnNormalizedPosition(),
//                            new Pose2DUnNormalized(DistanceUnit.MM, 250, 470, UnnormalizedAngleUnit.DEGREES, -45),
//                            0.5, 0))
//                    {
//                        telemetry.addLine("Score Sample 1");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    telemetry.addLine("Lift Arm!");
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        telemetry.addLine("Slide arm and Pivot Intake!");
//                    }
//                    if (this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.5);//Score in High Basket #1
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_1;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_1:
//                    holdTimer.reset();
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4,2.0))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        telemetry.addLine("Ready for Drive two");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_2;
//                    }
//                    break;
//
//
//                case DRIVE_TO_SAMPLE_2:  //drive to the Sample 2
//                     this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                     telemetry.addLine("Arm Down");
//                     if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                     {
//                        telemetry.addLine("Drive two should be happening");
//                        this.m_liftArmSubsystem.liftSlow(kLiftArmCloseSample);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                    }
//                    if (nav.driveTo(m_odo.getUnNormalizedPosition(), PrePickUpSample2, 0.4, .5))
//                    {
//                        telemetry.addLine("Ready to pick up Sample 2!");
//                        stateMachine = StateMachine.PICKUP_SAMPLE_2;
//                    }
//                break;
//
//
//                case PICKUP_SAMPLE_2:
//                    telemetry.addLine("Yes!");
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), PickUpSample2, 0.3, 0))
//                    {
//                        telemetry.addLine("Picked up Sample 2!");
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                        telemetry.addLine("Stopped Intake");
//                        stateMachine = StateMachine.SCORE_SAMPLE_2;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_2:
//                    if (nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4, 0))
//                    {
//                        telemetry.addLine("Score Sample 1");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    telemetry.addLine("Lift Arm!");
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        telemetry.addLine("Slide arm and Pivot Intake!");
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.5);//Score in High Basket #1
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_2;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_2:
//                    holdTimer.reset();
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4,2.0))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//
//                        telemetry.addLine("Ready to drive 3");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_3;
//                    }
//                    break;
//
//
//                case DRIVE_TO_SAMPLE_3:  //drive to the Sample 2
//                    this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                    telemetry.addLine("Arm Down");
//                    if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                {
//                    telemetry.addLine("Drive three should be happening");
//                    this.m_liftArmSubsystem.liftSlow(kLiftArmCloseSample);
//                    this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                    this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                }
//                if (nav.driveTo(m_odo.getUnNormalizedPosition(), PrePickUpSample3, 0.4, .5))
//                {
//                    telemetry.addLine("Ready to pick up Sample 3!");
//                    stateMachine = StateMachine.PICKUP_SAMPLE_3;
//                }
//                break;
//
//
//                case PICKUP_SAMPLE_3:
//                    telemetry.addLine("Yes!");
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), PickUpSample3, 0.3, 0))
//                    {
//                        telemetry.addLine("Picked up Sample 3!");
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                        telemetry.addLine("Stopped Intake");
//                        stateMachine = StateMachine.SCORE_SAMPLE_3;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_3:
//                    if (nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4, 0))
//                    {
//                        telemetry.addLine("Score Sample 3");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    telemetry.addLine("Lift Arm!");
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        telemetry.addLine("Slide arm and Pivot Intake!");
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.5);
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_3;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_3:
//                    holdTimer.reset();
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4,2.0))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        telemetry.addLine("Ready to drive to Sample 4");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_4;
//                    }
//                    break;
//
//
//                case DRIVE_TO_SAMPLE_4:  //drive to the Sample 4
//                    this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                    telemetry.addLine("Arm Down");
//                    if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                {
//                    telemetry.addLine("Drive FOUR should be happening");
//                    this.m_liftArmSubsystem.liftSlow(kLiftArmCloseSample);
//                    this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                    this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                }
//                if (nav.driveTo(m_odo.getUnNormalizedPosition(), PrePickUpSample4, 0.4, .5))
//                {
//                    telemetry.addLine("Ready to pick up Sample 4!");
//                    stateMachine = StateMachine.PICKUP_SAMPLE_4;
//                }
//                break;
//
//
//                case PICKUP_SAMPLE_4:
//                    telemetry.addLine("Yes!");
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), PickUpSample4, 0.5, .5))
//                    {
//                        telemetry.addLine("Picked up Sample 4!");
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                        telemetry.addLine("Stopped Intake");
//                        stateMachine = StateMachine.SCORE_SAMPLE_4;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_4:
//                    if (nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4, 0))
//                    {
//                        telemetry.addLine("Score Sample 4");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    telemetry.addLine("Lift Arm!");
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        telemetry.addLine("Slide arm and Pivot Intake!");
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.5);
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_4;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_4:
//                    holdTimer.reset();
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), NET_ZONE, 0.4,2.0))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                        telemetry.addLine("Arm Down");
//
//                        if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                        {
//                            this.m_liftArmSubsystem.liftSlow(kLiftArmCloseSample);
//                            this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        }
//                        telemetry.addLine("Ready to park");
//                        stateMachine = StateMachine.PRE_PARK;
//                    }
//                    break;
//
//
//                    case PRE_PARK:
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), PrePark,0.5,0.0))
//                    {
//                        this.m_ascentArmCommand.autoAscent();
//                        this.m_ascentArmSubsystem.ascentArm(0.78);
//                        telemetry.addLine("Almost There!");
//                        stateMachine = StateMachine.PARK_ASCENT_1;
//                    }
//                    break;
//
//
//                case PARK_ASCENT_1:
//                    if(nav.driveTo(m_odo.getUnNormalizedPosition(), ParkAscent1,0.5,0.0))
//                    {
//                        telemetry.addLine("Parked!");
//                        stateMachine = StateMachine.PARKED;
//                    }
//                    break;
//            }
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
//            Pose2DUnNormalized pos = m_odo.getUnNormalizedPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//
//            telemetry.update();
//
//        }
//    }
//}   // end class
//
