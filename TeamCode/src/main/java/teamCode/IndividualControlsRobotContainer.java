//package teamCode;
//
//import static teamCode.PoseStorage.currentPose;
//import static teamCode.PoseStorage.odoHeading;
//import static teamCode.PoseStorage.xEncoder;
//import static teamCode.PoseStorage.yEncoder;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.button.Button;
//import com.arcrobotics.ftclib.command.button.GamepadButton;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//
//import teamCode.commands.AimingOnCommand;
//import teamCode.commands.AimingOffCommand;
//import teamCode.commands.AimingTestingCommand;
//import teamCode.commands.LauncherServoCommand;
//import teamCode.commands.BumperParkingCommand;
//import teamCode.commands.ColorModeOnCommand;
//import teamCode.commands.DeParkingCommand;
//import teamCode.commands.DriveFieldOrientedCommand;
//import teamCode.commands.DriveManateeModeCommand;
//import teamCode.commands.IntakeModeCommand;
//import teamCode.commands.LauncherCommand;
//import teamCode.commands.LightCommand;
//import teamCode.commands.ParkingFudgeCommand;
//import teamCode.commands.ResetGyroCommand;
//import teamCode.commands.ScoreTestingCommand;
//import teamCode.commands.SorterOnCommand;
//import teamCode.commands.SorterOffCommand;
//import teamCode.commands.TimerCommand;
//import teamCode.commands.TransferServoOffCommand;
//import teamCode.commands.TransferServoOnCommand;
//import teamCode.commands.TurnTableCommand;
//import teamCode.subsystems.AimingServoSubsystem;
//import teamCode.subsystems.AxeSubsystem;
//import teamCode.subsystems.ColorSensorSubsystem;
//import teamCode.subsystems.DriveSubsystem;
//import teamCode.subsystems.GamepadSubsystem;
//import teamCode.subsystems.GimlisBoxMotorSubsystem;
//import teamCode.subsystems.GyroSubsystem;
//import teamCode.subsystems.HuskyLensSubsystem;
//import teamCode.subsystems.IntakeServoSubsystem;
//import teamCode.subsystems.LauncherSubsystem;
//import teamCode.subsystems.LightSubsystem;
//import teamCode.subsystems.SorterServoSubsystem;
//import teamCode.subsystems.TransferSubsystem;
//import teamCode.subsystems.TurnTableSubsystem;
//
//@TeleOp(name = "Individual Controls")
//public class IndividualControlsRobotContainer extends CommandOpMode
//{
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
//    private Button m_leftBumper;
//    private Button m_rightBumper;
//    private Button m_xButton;
//    private Button m_circle;
//    private Button m_square;
//    private Button m_triangle;
//    private Button m_dpadTop;
//    private Button m_dpadBottom;
//    private Button m_dpadLeft;
//    private Button m_dpadRight;
//    private Button m_gyroResetButton;
//    private Button m_manateeButton;
//    private Button m_scoreButton;
//    private Button m_axeButton;
//
//    /* Motors */
//    public DcMotor leftFront;
//    public DcMotor rightFront;
//    public DcMotor leftBack;
//    public DcMotor rightBack;
//    public DcMotor m_turnTableMotor;
//    public DcMotor m_parkingMotor;
//    public DcMotor m_launcherMotorRed;
//    public DcMotor m_launcherMotorBlue;
//
//
//    public CRServo m_intakeServo;
//    public Servo m_aimingServo;
//    public CRServo m_transferServo;
//    public CRServo m_sorterServo;
//    private Servo m_light;
//    private Servo m_AxeServo;
//
//    /* Sensors */
//    public HuskyLens m_huskylens;
//    public NormalizedColorSensor m_colorSensor;
//
//    /* Subsystems */
//    private DriveSubsystem m_driveSubsystem;
//    private AimingServoSubsystem m_aimingSubsystem;
//    private TurnTableSubsystem m_turnTableSubsystem;
//    private HuskyLensSubsystem m_huskyLensSubsystem;
//    private IntakeServoSubsystem m_intakeServoSubsystem;
//    private TransferSubsystem m_transferServoSubsystem;
//    private AxeSubsystem m_axeSubsystem;
//    private LauncherSubsystem m_launcherMotorSubsystem;
//    private SorterServoSubsystem m_sorterServoSubsystem;
//    private GimlisBoxMotorSubsystem m_parkingSubsystem;
//    private GyroSubsystem m_gyroSubsystem;
//    private GamepadSubsystem m_gamepadSubsystem;
//    private LightSubsystem m_lightSubsystem;
//    private ColorSensorSubsystem m_colorSensorSubsystem;
//    private ColorSensorSubsystem m_colorSubsystem;
//
//    /* Commands */
//    private DriveFieldOrientedCommand m_driveFieldOrientedCommand;
//    private IntakeModeCommand m_intakeWheelCommand;
//    private LauncherCommand m_launcherCommand;
//    private TransferServoOnCommand m_transferOnCommand;
//    private TransferServoOffCommand m_transferOffCommand;
//    private SorterOnCommand m_sorterOnCommand;
//    private SorterOffCommand m_sorterOffCommand;
//    private ColorModeOnCommand m_colorCommand;
//    private LightCommand m_lightCommand;
//    private LauncherServoCommand m_axeCommand;
//    private TurnTableCommand m_turnTableCommand;
//
//    private ParkingFudgeCommand m_parkingCommand;
//    private DeParkingCommand m_deParkingCommand;
//    private BumperParkingCommand m_bumperParkingCommand;
//    private AimingTestingCommand m_aimingTestCommand;
//    private AimingOnCommand m_aimingOnCommand;
//    private AimingOffCommand m_aimingOffCommand;
//    private DriveManateeModeCommand m_driveManateeModeCommand;
//    private ResetGyroCommand m_resetGyroCommand;
//    private ScoreTestingCommand m_scoreCommand;
//
//    private GoBildaPinpointDriver m_odo;
//    private TimerCommand m_timerCommand;
//
//    /* PID */
//    private PIDController m_pIDController;
//
//    @Override
//    public void initialize()
//    {
//        /* Drivetrain */
//        this.leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        this.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        this.leftBack = hardwareMap.get(DcMotor.class, "leftBack");
//        this.rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//
//        this.m_drive = new MecanumDrive
//                (
//                        new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
//                        new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
//                        new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312),
//                        new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312)
//                );
//
//        /* IMU */
//        this.m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        this.m_odo.setOffsets(87, -170, DistanceUnit.MM);//68,-178 are for Sting-Ray 3110-0002-0001 Product Insight #1
//        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.m_odo.setPosX(xEncoder,DistanceUnit.MM);
//        this.m_odo.setPosY(yEncoder,DistanceUnit.MM);
////        this.m_odo.setHeading(odoHeading,UnnormalizedAngleUnit.DEGREES);
//        this.m_odo.setHeading(odoHeading, AngleUnit.DEGREES);
//        double headingDegrees = currentPose.getHeading(UnnormalizedAngleUnit.DEGREES);
//
////                this.m_odo.setPosition(new Pose2DUnNormalized(DistanceUnit.MM,200,200,UnnormalizedAngleUnit.DEGREES,-45.0));
////                this.m_odo.setPosition(PoseStorage.poseStorage.xEncoder, PoseStorage.poseStorage.yEncoder,DistanceUnit.MM, PoseStorage.poseStorage.odoHeading,UnnormalizedAngleUnit.DEGREES);
//
//        System.out.println("Initial set");
//
////        System.out.println(m_odo.getUnNormalizedPosition());
//        System.out.println("Posi Variable");
//        System.out.println(xEncoder);
//        m_odo.recalibrateIMU();
//
//        float m_gain = 2;
//        final float[] m_hsvValues = new float[3];
//
//        /* Gamepad */
//
//        this.m_driver1 = new GamepadEx(gamepad1);
//        this.m_driver2 = new GamepadEx(gamepad2);
//
//        /* Motors */
//    //TODO Device Name MUST MATCH name on the Drivers Station!!!!!
//
//        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");
//        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
//        this.m_transferServo = new CRServo(hardwareMap, "transferServo");
//
//        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
//        this.m_launcherMotorRed = hardwareMap.get(DcMotor.class, "launcherMotorRed");
//        this.m_launcherMotorBlue = hardwareMap.get(DcMotor.class, "launcherMotorBlue");
//        this.m_parkingMotor = hardwareMap.get(DcMotor.class, "parkMotor");
//
//        this.m_light = hardwareMap.get(Servo.class, "light");
//
//        /* Sensors */
//        this.m_pIDController = new PIDController(0, 0, 0);
//        this.m_pIDController.setPID(0.0, 0.0, 0.0);
//        this.m_huskylens = hardwareMap.get(HuskyLens.class, "huskyLens");
//        this.m_huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        this.m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
//
//        /* Subsystems */
//
//        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
//        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
//        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2);
//        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
//        this.m_launcherMotorSubsystem = new LauncherSubsystem(this.m_launcherMotorRed/*,this.m_launcherMotorBlue*/);
//        this.m_parkingSubsystem = new GimlisBoxMotorSubsystem(this.m_parkingMotor);
//        this.m_huskyLensSubsystem = new HuskyLensSubsystem(this.m_huskylens);
//        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);
//        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
//        this.m_transferServoSubsystem = new TransferSubsystem(this.m_transferServo);
//        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
//        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");
//        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
//
//        register(this.m_driveSubsystem);
//        register(this.m_intakeServoSubsystem);
//
//        /* Default Commands */
//
//        this.m_driveFieldOrientedCommand = new DriveFieldOrientedCommand
//                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
//                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
//        this.m_driveSubsystem.setDefaultCommand(this.m_driveFieldOrientedCommand);
//
//        this.m_timerCommand = new TimerCommand (this.m_gamepadSubsystem, () -> getRuntime());
//        this.m_gamepadSubsystem.setDefaultCommand(this.m_timerCommand);
//
////        this.m_colorCommand = new ColorModeOnCommand(this.m_colorSensorSubsystem, this.m_axeSubsystem, this.m_transferServoSubsystem);
////        this.m_circle = (new GamepadButton(this.m_driver1, GamepadKeys.Button.Y))
////                .whenPressed(this.m_colorCommand);
//        /* Color Command with button in case next chunk of coding doesn't work */
//
////        this.m_colorCommand = new ColorModeOnCommand(this.m_colorSubsystem);
////        this.m_colorSubsystem.setDefaultCommand(this.m_colorCommand);
//        /* Makes it always display the color it sees */
//
////        this.m_intakeWheelCommand = new IntakeWheelCommand(this.m_intakeServoSubsystem, () -> this.m_driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
////                () -> this.m_driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
////        this.m_intakeServoSubsystem.setDefaultCommand(this.m_intakeWheelCommand);
//
//        this.m_turnTableCommand = new TurnTableCommand(this.m_turnTableSubsystem, () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
//                () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        this.m_turnTableSubsystem.setDefaultCommand(this.m_turnTableCommand);
//
//
//        schedule();
//        /* Event Commands */
//
//        this.m_resetGyroCommand = new ResetGyroCommand(this.m_gyroSubsystem);
//        this.m_gyroResetButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.START))
//                .whenPressed(this.m_resetGyroCommand);
//
//        this.m_bumperParkingCommand = new BumperParkingCommand(this.m_parkingSubsystem, () -> this.m_driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
//                () -> this.m_driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        this.m_parkingSubsystem.setDefaultCommand(this.m_bumperParkingCommand);
//
//
////        this.m_deParkingCommand = new DeParkingCommand(this.m_parkingSubsystem);
////        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
////                .whileHeld(this.m_deParkingCommand);
//
////       this.m_bumperParkingCommand = new BumperParkingCommand(this.m_parkingSubsystem);
////       this.m_rightBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
////               .whileHeld(this.m_bumperParkingCommand);
//
//
//        this.m_transferOffCommand = new TransferServoOffCommand(this.m_transferServoSubsystem);
//        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
//                .whenPressed(this.m_transferOffCommand);
//
//        this.m_aimingOnCommand = new AimingOnCommand(this.m_huskyLensSubsystem, this.m_turnTableSubsystem);
//        this.m_dpadTop = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_UP))
//                .whenPressed(this.m_aimingOnCommand);
//
//        this.m_aimingOffCommand = new AimingOffCommand(this.m_huskyLensSubsystem, this.m_turnTableSubsystem);
//        this.m_dpadBottom = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_DOWN))
//                .whenPressed(this.m_aimingOffCommand);
//
//        this.m_sorterOffCommand = new SorterOffCommand(this.m_sorterServoSubsystem);
//        this.m_leftBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_BUMPER))
//                .whenPressed(this.m_sorterOffCommand);
//
//        this.m_sorterOnCommand = new SorterOnCommand(this.m_sorterServoSubsystem);
//        this.m_rightBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.RIGHT_BUMPER))
//                .whenPressed(this.m_sorterOnCommand);
//
//        this.m_transferOnCommand = new TransferServoOnCommand(this.m_transferServoSubsystem);
//        this.m_square = (new GamepadButton(this.m_driver2, GamepadKeys.Button.X))
//                .whenPressed(this.m_transferOnCommand);
//
//        this.m_transferOffCommand = new TransferServoOffCommand(this.m_transferServoSubsystem);
//        this.m_circle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.B))
//                .whenPressed(this.m_transferOffCommand);
//
//        this.m_launcherCommand = new LauncherCommand(this.m_launcherMotorSubsystem, this.m_axeSubsystem);
//        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
//                .whenPressed(this.m_launcherCommand);
//
//        this.m_axeCommand = new LauncherServoCommand(this.m_axeSubsystem);
//        this.m_axeButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_STICK_BUTTON))
//                .whenPressed(this.m_axeCommand);
//    }
//
////    {
////        telemetry.addLine()
////                .addData("Red", "%.3f", m_colorSubsystem.getRed())
////                .addData("Green", "%.3f", m_colorSubsystem.getRed())
////                .addData("Blue", "%.3f", m_colorSubsystem.getRed());
////        telemetry.addLine()
////                .addData("Hue", "%.3f", m_hsvValues)
////                .addData("Saturation", "%.3f", m_hsvValues[1])
////                .addData("Value", "%.3f", hsvValues[2]);
////        telemetry.addData("Alpha", "%.3f", m_colorSensor.alpha());
////    }
//
////  {
////        for (int i = 1; i>0; i+=0)
////        {
////            telemetry.addData("Lift Arm", this.m_liftArmMotor.getCurrentPosition());
////            telemetry.addData("Slide Arm", this.m_slideArmMotor.getCurrentPosition());
////            telemetry.update();
////        }
//
////    }
////{
////        for (int i = 1; i>0; i+=0)
////        {
////           telemetry.addData("SorterServo", this.m_sorterServo.
////            telemetry.update();
////        }
//
////    }
//}
