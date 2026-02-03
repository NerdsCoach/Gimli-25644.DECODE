package teamCode;

import static teamCode.PoseStorage.currentPose;
import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.commands.AimingOnCommand;
import teamCode.commands.AimingTestingCommand;
import teamCode.commands.BellyOfTheBeastCommand;
import teamCode.commands.ColorModeOnCommand;

import teamCode.commands.DeParkingCommand;
import teamCode.commands.DefenseDownCommand;
import teamCode.commands.DefenseUpCommand;
import teamCode.commands.FudgeDeParkingCommand;
import teamCode.commands.HoodTestingCommand;
import teamCode.commands.LauncherOffCommand;
import teamCode.commands.LauncherOnCommand;
import teamCode.commands.OutTakeModeCommand;
import teamCode.commands.ParkingCommand;
import teamCode.commands.DriveFieldOrientedCommand;
import teamCode.commands.DriveManateeModeCommand;
import teamCode.commands.HoodDownCommand;
import teamCode.commands.HoodUpCommand;
import teamCode.commands.IntakeModeCommand;
import teamCode.commands.FudgeParkingCommand;
import teamCode.commands.RedLightCommand;
import teamCode.commands.ResetGyroCommand;
import teamCode.commands.ResetTurnTableCommand;
import teamCode.commands.ReverseTransferCommand;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferLimitCommand;

import teamCode.commands.AimingOffCommand;
import teamCode.commands.TurnTableLeftCommand;
import teamCode.commands.TurnTableRightCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.TurnTableSubsystem;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.ParkingSubsystem;
import teamCode.subsystems.GamepadSubsystem;
import teamCode.subsystems.GyroSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;

@TeleOp(name = "BLUE-DECODE")
public class RobotContainer extends CommandOpMode
{

    private ElapsedTime timer;

    /* Drivetrain */
    private MecanumDrive m_drive;

    /* IMU */
    private IMU m_imu;
    private IMU.Parameters m_imuParameters;

    /* Gamepad */
    private GamepadEx m_driver1;
    private GamepadEx m_driver2;

    private Button m_leftBumper;
    private Button m_rightBumper;
    private Button m_xButton;
    private Button m_circle;
    private Button m_square;
    private Button m_triangle;
    private Button m_dpadTop;
    private Button m_dpadBottom;
    private Button m_dpadLeft;
    private Button m_dpadRight;
    private Button m_gyroResetButton;
    private Button m_manateeButton;
    private Button m_scoreButton;
    private Button m_axeButton;
    private Button m_leftJoyStick;
    private Button m_rightTrigger;
    private Trigger m_leftTrigger;

    /* Motors*/
    //TODO TRIED PRIVATE INSTEAD OF PUBLIC, IT GAVE THE USAGES BUT IT STILL DIDN'T WORK....

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public DcMotor m_turnTableMotor;
    public DcMotor m_parkMotor;
    public DcMotorEx m_launcherMotorRed;

    public CRServo m_intakeServo;
    public Servo m_hoodServo;
    public CRServo m_transferServo;
    public CRServo m_sorterServo;
    private Servo m_light;
    private Servo m_AxeServo;

    /* Sensors */
    public HuskyLens m_huskylens;
    public NormalizedColorSensor m_colorSensor;
    private RevTouchSensor m_limitSwitch;


    /* Subsystems */
    private DriveSubsystem m_driveSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private HuskyLensSubsystem m_huskyLensSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;
//    private TransferSubsystem m_transferServoSubsystem;
    private AxeSubsystem m_axeSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private LauncherSubsystem m_launcherMotorSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ParkingSubsystem m_parkingSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private GamepadSubsystem m_gamepadSubsystem;
    private LightSubsystem m_lightSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;

    /* Commands */
    private DriveFieldOrientedCommand m_driveFieldOrientedCommand;
    private IntakeModeCommand m_intakeModeCommand;
    private OutTakeModeCommand m_outTakeModeCommand;
    private BellyOfTheBeastCommand m_bellyOfTheBeastCommand;
    private LauncherOnCommand m_launcherOnCommand;
    private LauncherOffCommand m_launcherOffCommand;
    private TurnTableLeftCommand m_turnTableLeftCommand;
    private ResetTurnTableCommand m_resetTurnTableCommand;
    private TurnTableRightCommand m_turnTableRightCommand;
    private ColorModeOnCommand m_colorOnCommand;
    private TransferLimitCommand m_transferLimitCommand;
    private ReverseTransferCommand m_reverseTransferCommand;


    private FudgeParkingCommand m_fudgeParkingCommand;
    private FudgeDeParkingCommand m_fudgeDeParkingCommand;
    private ParkingCommand m_parkingCommand;
    private DeParkingCommand m_deParkingCommand;
    private DefenseDownCommand m_defenseDownCommand;
    private DefenseUpCommand m_defenseUpCommand;
    private AimingTestingCommand m_aimingTestCommand;
    private AimingOnCommand m_aimingCommand;
    private AimingOffCommand m_turnOffAimingCommand;
    private DriveManateeModeCommand m_driveManateeModeCommand;
    private ResetGyroCommand m_resetGyroCommand;
    private HoodDownCommand m_hoodCloseCommand;
    private HoodUpCommand m_hoodFarCommand;
    private HoodTestingCommand m_hoodTestingCommand;

    private GoBildaPinpointDriver m_odo;
    private TimerCommand m_timerCommand;
    private RedLightCommand m_redLightCommand;

    /* PID */
    private PIDController m_pIDController;

    @Override
    public void initialize()
    {

        /* Drivetrain */
        this.leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        this.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        this.leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        this.rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        this.m_drive = new MecanumDrive
                (
                        new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312),
                        new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312)
                );

        /* IMU */
        this.m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.m_odo.setOffsets(87, -170, DistanceUnit.MM);//68,-178 are for Sting-Ray 3110-0002-0001 Product Insight #1
        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.m_odo.setPosX(xEncoder,DistanceUnit.MM);
        this.m_odo.setPosY(yEncoder,DistanceUnit.MM);
//        this.m_odo.setHeading(odoHeading,UnnormalizedAngleUnit.DEGREES);
        this.m_odo.setHeading(odoHeading, AngleUnit.DEGREES);
        double headingDegrees = currentPose.getHeading(UnnormalizedAngleUnit.DEGREES);

        //TODO TRIED ADDING IMU, STILL DOESN'T WORK...

//                this.m_odo.setPosition(new Pose2DUnNormalized(DistanceUnit.MM,200,200,UnnormalizedAngleUnit.DEGREES,-45.0));
//                this.m_odo.setPosition(PoseStorage.poseStorage.xEncoder, PoseStorage.poseStorage.yEncoder,DistanceUnit.MM, PoseStorage.poseStorage.odoHeading,UnnormalizedAngleUnit.DEGREES);

        System.out.println("Initial set");

//        System.out.println(m_odo.getUnNormalizedPosition());
        System.out.println("Posi Variable");
        System.out.println(xEncoder);
        m_odo.recalibrateIMU();

        float m_gain = 2;
        final float[] m_hsvValues = new float[3];

        /* Gamepad */

        this.m_driver1 = new GamepadEx(gamepad1);
        this.m_driver2 = new GamepadEx(gamepad2);



        /* Motors */
    //TODO Device Name MUST MATCH name on the Drivers Station!!!!!

        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");
        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");

        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        this.m_launcherMotorRed = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_parkMotor = hardwareMap.get(DcMotor.class, "parkMotor");

        this.m_light = hardwareMap.get(Servo.class, "light");

        /* Sensors */
        this.m_pIDController = new PIDController(0, 0, 0);
        this.m_pIDController.setPID(0.0, 0.0, 0.0);
        this.m_huskylens = hardwareMap.get(HuskyLens.class, "huskyLens");
        this.m_huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        this.m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");

        /* Subsystems */

        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2, this.m_light); //
        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_launcherMotorSubsystem = new LauncherSubsystem(this.m_launcherMotorRed);
        this.m_parkingSubsystem = new ParkingSubsystem(this.m_parkMotor);
        this.m_huskyLensSubsystem = new HuskyLensSubsystem(this.m_huskylens, 4);//TODO Check this works
        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);
        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
        this.m_limitSwitchSubsystem = new LimitSwitchSubsystem(this.m_limitSwitch, this.m_transferServo);
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "aimingServo");

        register(this.m_driveSubsystem);
        register(this.m_intakeServoSubsystem);
        register(this.m_gamepadSubsystem);
        register(this.m_turnTableSubsystem); //Added this TODO

        /* Default Commands */

        //DRIVER

        this.m_driveFieldOrientedCommand = new DriveFieldOrientedCommand
                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
                        () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
        this.m_driveSubsystem.setDefaultCommand(this.m_driveFieldOrientedCommand);

//        this.m_driveManateeModeCommand = new DriveManateeModeCommand
//                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
//                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
//
        this.m_timerCommand = new TimerCommand (this.m_gamepadSubsystem, () -> getRuntime());
        this.m_gamepadSubsystem.setDefaultCommand(this.m_timerCommand);

        this.m_redLightCommand = new RedLightCommand(this.m_gamepadSubsystem,() -> getRuntime() ,this.m_lightSubsystem);

        schedule();

        //DRIVER
        this.m_resetGyroCommand = new ResetGyroCommand(this.m_gyroSubsystem);
        this.m_gyroResetButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.START))
                .whenPressed(this.m_resetGyroCommand);

        this.m_intakeModeCommand = new IntakeModeCommand(this.m_intakeServoSubsystem/*, this.m_sorterServoSubsystem*/);
        this.m_rightBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_intakeModeCommand);

        this.m_outTakeModeCommand = new OutTakeModeCommand(this.m_intakeServoSubsystem);
        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(this.m_outTakeModeCommand);

        this.m_fudgeParkingCommand = new FudgeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadLeft = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_LEFT))
                .whileHeld(this.m_fudgeParkingCommand);

        this.m_fudgeDeParkingCommand = new FudgeDeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadRight = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_RIGHT))
                .whileHeld(this.m_fudgeDeParkingCommand);

        this.m_parkingCommand = new ParkingCommand(this.m_parkingSubsystem, this.m_intakeServoSubsystem, this.m_launcherMotorSubsystem, this.m_sorterServoSubsystem, this.m_turnTableSubsystem);
        this.m_dpadTop = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_UP))
                .whenPressed(this.m_parkingCommand);

        this.m_deParkingCommand = new DeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadBottom = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(this.m_deParkingCommand);

        this.m_defenseUpCommand = new DefenseUpCommand(this.m_parkingSubsystem);
        this.m_xButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.A))
                .whenPressed(this.m_defenseUpCommand);

        this.m_defenseDownCommand = new DefenseDownCommand(this.m_parkingSubsystem);
        this.m_triangle = (new GamepadButton(this.m_driver1, GamepadKeys.Button.Y))
                .whenPressed(this.m_defenseDownCommand);


        //GADGETEER

        //TODO make color into a toggle
        //TODO it only reads the color once after pressing button

        this.m_colorOnCommand = new ColorModeOnCommand(this.m_colorSensorSubsystem);
        this.m_leftBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(this.m_colorOnCommand);

        this.m_bellyOfTheBeastCommand = new BellyOfTheBeastCommand(this.m_sorterServoSubsystem);
        this.m_triangle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.Y))
                .whenPressed(this.m_bellyOfTheBeastCommand);

        this.m_launcherOnCommand = new LauncherOnCommand(this.m_launcherMotorSubsystem, this.m_axeSubsystem, this.m_huskyLensSubsystem, this.m_hoodServoSubsystem);
        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
                .whenPressed(this.m_launcherOnCommand);

        this.m_launcherOffCommand = new LauncherOffCommand(this.m_launcherMotorSubsystem, this.m_axeSubsystem, this.m_huskyLensSubsystem);
        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.START))
                .whenPressed(this.m_launcherOffCommand);

//        this.m_turnTableLeftCommand = new TurnTableLeftCommand(this.m_turnTableSubsystem);
//        this.m_dpadLeft = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_LEFT))
//                .whileHeld(this.m_turnTableLeftCommand);

        this.m_resetTurnTableCommand = new ResetTurnTableCommand(this.m_turnTableSubsystem);
        this.m_dpadLeft = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(this.m_resetTurnTableCommand);

//        this.m_turnTableRightCommand = new TurnTableRightCommand(this.m_turnTableSubsystem);
//        this.m_dpadRight = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_RIGHT))
//                .whileHeld(this.m_turnTableRightCommand);

        this.m_aimingCommand = new AimingOnCommand(this.m_huskyLensSubsystem, this.m_turnTableSubsystem);
        this.m_square = (new GamepadButton(this.m_driver2, GamepadKeys.Button.X))
                .whenPressed(this.m_aimingCommand);

        this.m_turnOffAimingCommand = new AimingOffCommand(this.m_huskyLensSubsystem, this.m_turnTableSubsystem, this.m_colorSensorSubsystem);
        this.m_circle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.B))
                .whenPressed(this.m_turnOffAimingCommand);

        this.m_hoodCloseCommand = new HoodDownCommand(this.m_hoodServoSubsystem);
        this.m_dpadBottom = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(this.m_hoodCloseCommand);

        this.m_hoodFarCommand = new HoodUpCommand(this.m_hoodServoSubsystem);
        this.m_dpadTop = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_UP))
                .whenPressed(this.m_hoodFarCommand);

        this.m_reverseTransferCommand = new ReverseTransferCommand(this.m_limitSwitchSubsystem, () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        this.m_leftTrigger = new Trigger(() -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05);
        m_leftTrigger.whileActiveContinuous(m_reverseTransferCommand);

        this.m_transferLimitCommand = new TransferLimitCommand(this.m_limitSwitchSubsystem);
        this.m_rightBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_transferLimitCommand);

//        this.m_driveManateeModeCommand = new DriveManateeModeCommand(this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
//                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
//        this.m_manateeButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.B))
//                .toggleWhenPressed(this.m_driveFieldOrientedCommand, this.m_driveManateeModeCommand);

//        this.m_pose2DObservationZoneCommand = new Pose2DObservationZoneCommand(this.m_driveSubsystem, this.m_odo,
//                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
//        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(this.m_pose2DObservationZoneCommand);

    }
}
