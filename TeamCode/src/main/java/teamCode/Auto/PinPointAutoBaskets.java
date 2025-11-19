//package teamCode.Auto;
//
//import static teamCode.Constants.LiftArmConstants.kLiftArmCloseSample;
//import static teamCode.Constants.LiftArmConstants.kLiftArmHighBasket;
//import static teamCode.Constants.LiftArmConstants.kLiftArmIntakeReset;
//import static teamCode.Constants.SorterConstants.kIntakePivotPickUp;
//import static teamCode.Constants.SorterConstants.kIntakePivotScore;
//import static teamCode.Constants.SlideArmConstants.kSlideArmCloseSample;
//import static teamCode.Constants.SlideArmConstants.kSlideArmHighBasket;
//import static teamCode.Constants.SlideArmConstants.kSlideArmHome;
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
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import teamCode.DriveToPoint;
//
//import java.util.Locale;
//
//import teamCode.Pose2DUnNormalized;
//import teamCode.commands.StingrayAscent1ArmCommand;
//import teamCode.subsystems.PurplePotatoSorterSubsystemOG;
//import teamCode.subsystems.IntakeServoSubsystem;
//import teamCode.subsystems.LiftArmSubsystem;
//import teamCode.subsystems.BilbosTurnTableSubsystem;
//import teamCode.subsystems.StingRayArmSubsystem;
//
//@Autonomous(name="Pin Point Auto Baskets", group="Pinpoint")
////@Disabled
//
//public class PinPointAutoBaskets extends LinearOpMode
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
//    private StingrayAscent1ArmCommand m_ascentArmCommand;
//    private TouchSensor m_touch;
//    private final ElapsedTime holdTimer = new ElapsedTime();
//
//    GoBildaPinpointDriver m_odo; // Declare OpMode member for the Odometry Computer
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
//        PARKED, PICKUP_SAMPLE_4_TURN, PICKUP_SAMPLE_4_DRIVE,
//    }
////                                                                                x=250, y=470
//    static final Pose2DUnNormalized NET_ZONE = new Pose2DUnNormalized(DistanceUnit.MM, 240, 480, UnnormalizedAngleUnit.DEGREES, -45);
//    static final Pose2DUnNormalized PrePickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 400, 420, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PickUpSample2 = new Pose2DUnNormalized(DistanceUnit.MM, 643, 420, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PrePickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 400, 675, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PickUpSample3 = new Pose2DUnNormalized(DistanceUnit.MM, 680, 675, UnnormalizedAngleUnit.DEGREES, 0);
//    static final Pose2DUnNormalized PrePickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 600, 570, UnnormalizedAngleUnit.DEGREES, 52);
//    static final Pose2DUnNormalized PickUpSample4 = new Pose2DUnNormalized(DistanceUnit.MM, 670, 675, UnnormalizedAngleUnit.DEGREES, 50);
//    static final Pose2DUnNormalized PickUpSample4Turn = new Pose2DUnNormalized(DistanceUnit.MM, 760, 690, UnnormalizedAngleUnit.DEGREES, 40);
//    static final Pose2DUnNormalized PickUpSample4Drive = new Pose2DUnNormalized(DistanceUnit.MM, 850, 690, UnnormalizedAngleUnit.DEGREES, 45);
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
//        m_odo.resetPosAndIMU();
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
//        this.m_intakeWheelSubsystem = new IntakeServoSubsystem(this.m_intakeWheelServo
//        );
//        this.m_ascentArmCommand = new StingrayAscent1ArmCommand(this.m_ascentArmSubsystem);
//
//
//        this.m_slideArmSubsystem.slideArm(kSlideArmHome);
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
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(1.0);//Score in High Basket #1
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_1;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_1:
//                    holdTimer.reset();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            0.4,1.75))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_2;
//                    }
//                    break;
//
//
//                case DRIVE_TO_SAMPLE_2:  //drive to the Sample 2
//                     this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                     if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                     {
//                        this.m_liftArmSubsystem.liftArm(kLiftArmCloseSample);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                    }
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PrePickUpSample2, 0.4, .5))
//                    {
//                        stateMachine = StateMachine.PICKUP_SAMPLE_2;
//                    }
//                break;
//
//
//                case PICKUP_SAMPLE_2:
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample2, 0.3, 0))
//                    {
//                        telemetry.addLine("Picked up Sample 2!");
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                        stateMachine = StateMachine.SCORE_SAMPLE_2;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_2:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.45, 0))
//                    {
//                        telemetry.addLine("Score Sample 2 ");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(1.0);//Score in High Basket #1
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_2;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_2:
//                    holdTimer.reset();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            0.45,2.5))
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
//                    if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                {
//                    telemetry.addLine("Drive three should be happening");
//                    this.m_liftArmSubsystem.liftArm(kLiftArmCloseSample);
//                    this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                    this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                }
//                if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                        PrePickUpSample3, 0.4, .5))
//                {
//                    stateMachine = StateMachine.PICKUP_SAMPLE_3;
//                }
//                break;
//
//
//                case PICKUP_SAMPLE_3:
//                    resetRuntime();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample3, 0.4, 0)|| getRuntime() > 2)
//                    {
//                        telemetry.addLine("Picked up Sample 3!");
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_liftArmSubsystem.liftArm(kLiftArmIntakeReset);
//                        stateMachine = StateMachine.SCORE_SAMPLE_3;
//                    }
//                    break;
//
//
//                case SCORE_SAMPLE_3:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.4, 0))
//                    {
//                        telemetry.addLine("Score Sample 3");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(1.0);
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_3;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_3:
//                    holdTimer.reset();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            0.4,2))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        telemetry.addLine("Ready to drive to Sample 4");
//                        stateMachine = StateMachine.DRIVE_TO_SAMPLE_4;
//                    }
//                    break;
//
//
//
//                case DRIVE_TO_SAMPLE_4:  //drive to the Sample 4
//                    this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//                    if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                {
//                    telemetry.addLine("Drive FOUR should be happening");
//                    this.m_liftArmSubsystem.liftArm(kLiftArmCloseSample);
//                    this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                    this.m_intakeWheelSubsystem.spinIntake(-1.0);//Turn On intake
//                }
//                if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                        PrePickUpSample4, 0.5, 0))
//                {
//                    telemetry.addLine("Ready to pick up Sample 4!");
//                    stateMachine = StateMachine.PICKUP_SAMPLE_4;
//                }
//                break;
//
//                case PICKUP_SAMPLE_4:
//                    resetRuntime();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample4, 0.5, 0)|| getRuntime() > 3)
//                    {
//                        stateMachine = StateMachine.PICKUP_SAMPLE_4_TURN;
//                    }
//                    break;
//
//                case PICKUP_SAMPLE_4_TURN:
//                    resetRuntime();
//                    telemetry.addLine("347!");
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample4Turn, 0.5, 0) || getRuntime() > 3)
//                    {
//                        stateMachine = StateMachine.PICKUP_SAMPLE_4_DRIVE;
//                    }
//                    break;
//
//                case PICKUP_SAMPLE_4_DRIVE:
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PickUpSample4Drive, 0.6, .5) || getRuntime() >5)
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
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            NET_ZONE, 0.45, .3))
//                    {
//                        telemetry.addLine("Score Sample 4");
//                    }
//                    this.m_liftArmSubsystem.liftArm(kLiftArmHighBasket);
//                    if (this.m_liftArmSubsystem.atTarget(kLiftArmHighBasket))
//                    {
//                        this.m_slideArmSubsystem.slideArm(kSlideArmHighBasket);
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                    }
//                    if(this.m_slideArmSubsystem.atTarget(kSlideArmHighBasket))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(1.0);
//                        telemetry.addLine("Score!");
//                        stateMachine = StateMachine.TURN_OFF_INTAKE_4;
//                    }
//                    break;
//
//
//                case TURN_OFF_INTAKE_4:
//                    holdTimer.reset();
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            0.45,2))
//                    {
//                        this.m_intakeWheelSubsystem.spinIntake(0.0);//Stop intake
//                        this.m_intakePivotSubsystem.SpinSorter(kIntakePivotPickUp);
//                        this.m_slideArmSubsystem.slideArm(kSlideArmCloseSample);
//
//                        if (this.m_slideArmSubsystem.atTarget(kSlideArmCloseSample));
//                        {
//                            this.m_liftArmSubsystem.liftArm(kLiftArmCloseSample);
//                            this.m_intakePivotSubsystem.SpinSorter(kIntakePivotScore);
//                        }
//                        telemetry.addLine("Ready to park");
//                        stateMachine = StateMachine.PRE_PARK;
//                    }
//                    break;
//
//
//                    case PRE_PARK:
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            PrePark,0.5,0.0))
//                    {
//                        this.m_ascentArmCommand.autoAscent();
//                        this.m_ascentArmSubsystem.ascentArm(0.7);
//                        telemetry.addLine("Almost There!");
//                        stateMachine = StateMachine.PARK_ASCENT_1;
//                    }
//                    break;
//
//
//                case PARK_ASCENT_1:
//                    if(nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            ParkAscent1,0.5,0.0))
//                    {
//                        telemetry.addLine("Parked!");
//                        stateMachine = StateMachine.PARKED;
//                        System.out.println("End Position");
//                        System.out.println(m_odo.getPosition());
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
//
//
////            PoseStorage.poseStorage.currentPose = m_odo.getUnNormalizedPosition();
//
//
//
//            Pose2DUnNormalized pos = new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES));
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
////            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//
//            telemetry.update();
//
//        }
//    }
//}   // end class
//
