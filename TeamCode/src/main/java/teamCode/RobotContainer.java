//package teamCode;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.button.Button;
//import com.arcrobotics.ftclib.command.button.GamepadButton;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import teamCode.commands.FlyWheelCommand;
//
//import teamCode.subsystems.FlyWheelSubsystem;
//
////Hello!!!
//
//@TeleOp(name = "Sting-Ray")
//public class RobotContainer extends CommandOpMode {
//
//    private ElapsedTime timer;
//
//    /* Drivetrain */
//    private MecanumDrive m_drive;
//
//    /* IMU */
//    private IMU m_imu;
//    private IMU.Parameters m_imuParameters;
//
//    /* Gamepad */
//    private GamepadEx m_driver1;
//    private GamepadEx m_driver2;
//
////    private Button m_leftBumper;
////    private Button m_rightBumper;
////    private Button m_xButton;
////    private Button m_circle;
////    private Button m_square;
////    private Button m_triangle;
////    private Button m_dpadTop;
////    private Button m_dpadBottom;
////    private Button m_dpadLeft;
////    private Button m_dpadRight;
////    private Button m_gyroResetButton;
////    private Button m_holdPositionButton;
////    private Button m_resetHomeButton;
////    private Button m_slideFudgeInButton;
////    private Button m_slideFudgeOutButton;
////    private Button m_releaseClimbButton;
////    private Button m_manateeButton;
//    private Button m_flyWheelButton;
//
//    /* Motors */
////    public DcMotor leftFront;
////    public DcMotor rightFront;
////    public DcMotor leftBack;
////    public DcMotor rightBack;
////    private DcMotor m_slideArmMotor;
////    private DcMotor m_liftArmMotor;
////
////    private CRServo m_intakeWheelServo;
////
////    /* Sensors */
////    private TouchSensor m_touch;
////    public DcMotor m_flyWheelsMotor;
//    public DcMotor m_leftMotor;
//    public DcMotor m_rightMotor;
//
//    /* Subsystems */
////    private DriveSubsystem m_driveSubsystem;
////    private BilbosTurnTableSubsystem m_slideArmSubsystem;
////    private LiftArmSubsystem m_liftArmSubsystem;
////    private PurplePotatoSorterSubsystemOG m_intakePivotSubsystem;
////    private SecondBreakfastIntakeSubsystem m_intakeWheelSubsystem;
////    private StingRayArmSubsystem m_ascentArmSubsystem;
////    private GyroSubsystem m_gyroSubsystem;
////    private GamepadSubsystem m_gamepadSubsystem;
////    private GrabBatSubsystem m_grabBatSubsystem;
//    private FlyWheelSubsystem m_flyWheelSubsystem;
//
//    /* Commands */
////    private DriveFieldOrientedCommand m_driveFieldOrientedCommand;
////    private DriveManateeModeCommand m_driveManateeModeCommand;
////
////    private ArmFudgeFactorUpCommand m_armFudgeFactorUpCommand;
////    private ArmFudgeFactorDownCommand m_armFudgeFactorDownCommand;
////    private ArmPositionCloseSampleCommand m_armPositionCloseSampleCommand;
////    private ArmPositionFarSampleCommand m_armPositionFarSampleCommand;
////    private ArmPositionMidSampleCommand m_armPositionSubmersiblePickUpCommand;
////    private ArmPositionHighBasketCommand m_armPositionHighBasketCommand;
////    private ArmPositionHighChamberCommand m_armPositionHighChamberCommand;
////    private ArmPositionLowBasketCommand m_armPositionLowBasketCommand;
////    private ArmPositionScoreHighChamberCommand m_armPositionScoreHighChamberCommand;
////    private ArmPositionTravelCommand m_armPositionTravelCommand;
////
////    private SlideFudgeInCommand m_slideFudgeInCommand;
////    private SlideFudgeOutCommand m_slideFudgeOutCommand;
////    private ClimbArmReleaseCommand m_releaseCLimberArmCommand;
////
////    private IntakePivotCommand m_intakePivotCommand;
////    private IntakeWheelCommand m_intakeWheelCommand;
////    private StingrayAscent1ArmCommand m_ascentArmCommand;
////
////    private Pose2DNetZoneCommand m_pose2DNetZoneCommand;
////    private Pose2DObservationZoneCommand m_pose2DObservationZoneCommand;
////    private Pose2DScoreChamberCommand m_pose2DScoreChamberCommand;
////    private Pose2DSubmersibleCommand m_pose2DSubmersibleCommand;
////
////    private ResetGyroCommand m_resetGyroCommand;
////    private ResetHomeCommand m_resetHomeCommand;
////    private HoldBasketPositionCommand m_holdBasketPositionCommand;
////    private HoldChamberPositionCommand m_holdChamberPositionCommand;
////
////    private GoBildaPinpointDriver m_odo;
////    private TimerCommand m_timerCommand;
////    double Runtime;
////    private GrabBatCommand m_grabBatCommand;
//    private FlyWheelCommand m_flyWheelCommand;
//
////    private com.qualcomm.robotcore.hardware.TouchSensor m_touchSensor;
////    private boolean touchSensorIsPressed = false;
//
//
//    /* PID */
//    private PIDController m_pIDController;
//
//    @Override
//    public void initialize()
//    {
//        /* Drivetrain */
////        this.leftFront = hardwareMap.get(DcMotor.class, "leftFront");
////        this.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
////        this.leftBack = hardwareMap.get(DcMotor.class, "leftBack");
////        this.rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//
////        this.m_drive = new MecanumDrive
////                (
////                        new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
////                        new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
////                        new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312),
////                        new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312)
////                );
////
////        /* IMU */
////        this.m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//////        this.m_odo.setOffsets(68, -178);//68,-178 these are tuned for Sting-Ray 3110-0002-0001 Product Insight #1
//////        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//////        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//////        this.m_odo.setPosX(xEncoder,DistanceUnit.MM);
//////        this.m_odo.setPosY(yEncoder,DistanceUnit.MM);
//////        this.m_odo.setHeading(odoHeading,UnnormalizedAngleUnit.DEGREES);
//////        this.m_odo.setPosition(new Pose2DUnNormalized(DistanceUnit.MM,200,200,UnnormalizedAngleUnit.DEGREES,-45.0));
////        //        this.m_odo.setPosition(PoseStorage.poseStorage.xEncoder, PoseStorage.poseStorage.yEncoder,DistanceUnit.MM, PoseStorage.poseStorage.odoHeading,UnnormalizedAngleUnit.DEGREES);
//
////        System.out.println("Initial set");
//////        System.out.println(m_odo.getUnNormalizedPosition());
////        System.out.println("Posi Variable");
////        System.out.println(PoseStorage.xEncoder);
//////        m_odo.recalibrateIMU();
//
//        /* Gamepad */
//
//        this.m_driver1 = new GamepadEx(gamepad1);
//        this.m_driver2 = new GamepadEx(gamepad2);
//
//        /* Motors */
//
////        this.m_flyWheelsMotor = hardwareMap.get(DcMotor.class, "flyWheelMotor");
//        this.m_leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        this.m_rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
////        this.m_slideArmMotor = hardwareMap.get(DcMotor.class, "slideArmMotor");
//////        this.m_liftArmMotor = hardwareMap.get(DcMotor.class, "liftArmMotor");
//////        this.m_intakeWheelServo = new CRServo(hardwareMap, "intakeWheelServo");
//
//        /* Sensors */
////        this.m_touchSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "intakeTouchSensor");
////        this.m_pIDController = new PIDController(0, 0, 0);
////        this.m_pIDController.setPID(0.0, 0.0, 0.0);
//
//        /* Subsystems */
//
////        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
////        this.m_slideArmSubsystem = new BilbosTurnTableSubsystem(this.m_slideArmMotor);
////        this.m_liftArmSubsystem = new LiftArmSubsystem(this.m_liftArmMotor)/*() -> this.m_pIDController.calculate(this.m_liftArmMotor.getCurrentPosition()))*/;
////        this.m_intakePivotSubsystem = new PurplePotatoSorterSubsystemOG(hardwareMap, "intakePivotServo");
////        this.m_intakeWheelSubsystem = new SecondBreakfastIntakeSubsystem(this.m_intakeWheelServo, this.m_touchSensor);
////        this.m_ascentArmSubsystem = new StingRayArmSubsystem(hardwareMap, "ascentArmServo");
////        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
////        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2);
////        this.m_grabBatSubsystem = new GrabBatSubsystem(hardwareMap, "grabBatServo");
//
//        this.m_flyWheelSubsystem = new FlyWheelSubsystem( this.m_leftMotor, this.m_rightMotor);
//
////        register(this.m_driveSubsystem);
////        register(this.m_intakeWheelSubsystem);
////        register(this.m_gamepadSubsystem);
////        register(this.m_gyroSubsystem);
//        register(this.m_flyWheelSubsystem);
//
//        /* Default Commands */
//
//
////
////        this.m_driveFieldOrientedCommand = new DriveFieldOrientedCommand
////                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
////                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
////        this.m_driveManateeModeCommand = new DriveManateeModeCommand
////
////                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
////                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
////        this.m_driveSubsystem.setDefaultCommand(this.m_driveFieldOrientedCommand);
////
////        this.m_timerCommand = new TimerCommand (this.m_gamepadSubsystem, () -> getRuntime());
////        this.m_gamepadSubsystem.setDefaultCommand(this.m_timerCommand);
////
////        schedule();
////
////        this.m_intakeWheelCommand = new IntakeWheelCommand
////                (this.m_intakeWheelSubsystem, () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
////                () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
////        this.m_intakeWheelSubsystem.setDefaultCommand(this.m_intakeWheelCommand);
//
//
//        /* Event Commands */
//
//        this.m_flyWheelCommand = new FlyWheelCommand(this.m_flyWheelSubsystem);
//        new GamepadButton(this.m_driver1, GamepadKeys.Button.A)
//                .whileHeld(this.m_flyWheelCommand);
//
////        this.m_manateeButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.A))
////                .toggleWhenPressed(this.m_driveFieldOrientedCommand, this.m_driveManateeModeCommand);
////
////        this.m_holdBasketPositionCommand = new HoldBasketPositionCommand(this.m_gyroSubsystem);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.BACK)
////                .whenPressed(this.m_holdBasketPositionCommand);
////
////        this.m_holdChamberPositionCommand = new HoldChamberPositionCommand(this.m_gyroSubsystem);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.START)
////                .whenPressed(this.m_holdChamberPositionCommand);
////
////        this.m_resetHomeCommand = new ResetHomeCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem);
////        this.m_resetHomeButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.START))
////                .whenPressed(this.m_resetHomeCommand);
////
////        this.m_slideFudgeInCommand = new SlideFudgeInCommand(m_slideArmSubsystem);
////        this.m_slideFudgeInButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.BACK))
////                .whileHeld(this.m_slideFudgeInCommand);
////
////        this.m_slideFudgeOutCommand = new SlideFudgeOutCommand(m_slideArmSubsystem);
////        this.m_slideFudgeOutButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_STICK_BUTTON))
////                .whileHeld(this.m_slideFudgeOutCommand);
////
////        this.m_armFudgeFactorUpCommand = new ArmFudgeFactorUpCommand(m_liftArmSubsystem);
////        this.m_dpadRight = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_RIGHT))
////                .whenPressed(this.m_armFudgeFactorUpCommand);
////
////        this.m_armFudgeFactorDownCommand = new ArmFudgeFactorDownCommand(m_liftArmSubsystem);
////        this.m_dpadLeft = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_LEFT))
////                .whenPressed(this.m_armFudgeFactorDownCommand);
////
////        this.m_armPositionCloseSampleCommand = new ArmPositionCloseSampleCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, this.m_intakePivotSubsystem);
////        this.m_square = (new GamepadButton(this.m_driver2, GamepadKeys.Button.X))
////                .whenPressed(this.m_armPositionCloseSampleCommand);
////
////        this.m_armPositionSubmersiblePickUpCommand = new ArmPositionMidSampleCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, m_intakePivotSubsystem);
////        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
////                .whenPressed(this.m_armPositionSubmersiblePickUpCommand);
////
////        this.m_armPositionFarSampleCommand = new ArmPositionFarSampleCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, m_intakePivotSubsystem);
////        this.m_circle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.B))
////                .whenPressed(this.m_armPositionFarSampleCommand);
////
////        this.m_armPositionHighBasketCommand = new ArmPositionHighBasketCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, m_intakePivotSubsystem);
////        this.m_triangle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.Y))
////                .whenPressed(this.m_armPositionHighBasketCommand);
////
////        this.m_armPositionHighChamberCommand = new ArmPositionHighChamberCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, this.m_intakePivotSubsystem);
////        this.m_dpadTop = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_UP))
////                .whenPressed(this.m_armPositionHighChamberCommand);
////
////        this.m_armPositionScoreHighChamberCommand = new ArmPositionScoreHighChamberCommand
////                (m_liftArmSubsystem, m_slideArmSubsystem, m_intakePivotSubsystem, m_intakeWheelSubsystem);
////        this.m_dpadBottom = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_DOWN))
////                .whenPressed(this.m_armPositionScoreHighChamberCommand);
////
////        this.m_armPositionTravelCommand = new ArmPositionTravelCommand
////                (this.m_liftArmSubsystem, this.m_slideArmSubsystem, this.m_intakePivotSubsystem);
////        this.m_leftBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_BUMPER))
////                .whenPressed(this.m_armPositionTravelCommand);
////
////        this.m_releaseCLimberArmCommand = new ClimbArmReleaseCommand(m_liftArmSubsystem);
////        this.m_releaseClimbButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.Y))
////                .whenPressed(this.m_releaseCLimberArmCommand);
////
////        this.m_pose2DNetZoneCommand = new Pose2DNetZoneCommand(this.m_driveSubsystem, this.m_odo,
////                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(this.m_pose2DNetZoneCommand);
////
////        this.m_pose2DSubmersibleCommand= new Pose2DSubmersibleCommand(this.m_driveSubsystem, this.m_odo,
////                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_UP).whenPressed(this.m_pose2DSubmersibleCommand);
////
////        this.m_pose2DScoreChamberCommand = new Pose2DScoreChamberCommand(this.m_driveSubsystem, this.m_odo,
////                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(this.m_pose2DScoreChamberCommand);
////
////        this.m_pose2DObservationZoneCommand = new Pose2DObservationZoneCommand(this.m_driveSubsystem, this.m_odo,
////                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
////        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(this.m_pose2DObservationZoneCommand);
////
////        this.m_intakePivotCommand = new IntakePivotCommand(this.m_intakePivotSubsystem);
////        this.m_rightBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.RIGHT_BUMPER))
////                .whenPressed(this.m_intakePivotCommand);
////
////        this.m_ascentArmCommand = new StingrayAscent1ArmCommand(this.m_ascentArmSubsystem);
////        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
////                .whenPressed(this.m_ascentArmCommand);
////
////        this.m_resetGyroCommand = new ResetGyroCommand(this.m_gyroSubsystem);
////        this.m_gyroResetButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
////                .whenPressed(this.m_resetGyroCommand);
//
////        this.m_grabBatCommand = new GrabBatCommand(this.m_grabBatSubsystem);
////        this.m_rightBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
////                .whenPressed(this.m_grabBatCommand);
//    }
////
////  {
////        for (int i = 1; i>0; i+=0)
////        {
////            telemetry.addData("Lift Arm", this.m_liftArmMotor.getCurrentPosition());
////            telemetry.addData("Slide Arm", this.m_slideArmMotor.getCurrentPosition());
////            telemetry.update();
////        }
////    }
//
//
//}
