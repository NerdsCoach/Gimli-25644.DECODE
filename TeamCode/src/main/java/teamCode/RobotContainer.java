package teamCode;

import static teamCode.PoseStorage.currentPose;
import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.commands.AimingOnCommand;
import teamCode.commands.BellyOfTheBeastCommand;
import teamCode.commands.ColorModeOnCommand;

import teamCode.commands.DeParkingCommand;
import teamCode.commands.FudgeDeParkingCommand;
import teamCode.commands.IntakeMotorCommand;
import teamCode.commands.IntakeServoCommand;
import teamCode.commands.LauncherBellyCommand;

import teamCode.commands.OutTakeMotorCommand;
import teamCode.commands.ParkingCommand;
import teamCode.commands.DriveFieldOrientedCommand;

import teamCode.commands.HoodUpCommand;
import teamCode.commands.FudgeParkingCommand;
import teamCode.commands.ResetGyroCommand;
import teamCode.commands.ResetTurnTableCommand;
import teamCode.commands.ReverseTransferCommand;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferLimitCommand;

import teamCode.commands.UnJamCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.LightASubsystem;
import teamCode.subsystems.LightBSubsystem;
import teamCode.subsystems.LimeLightSubsystem;
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

    private final ElapsedTime timer = new ElapsedTime();


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
    private Button m_optionsButton;
    private Button m_shareButton;
    private Button m_circle;
    private Button m_square;
    private Button m_triangle;
    private Button m_dpadTop;
    private Button m_dpadBottom;
    private Button m_dpadLeft;
    private Button m_dpadRight;
    private Button m_gyroResetButton;
    private Trigger m_leftTrigger;

    /* Motors*/
    //TODO TRIED PRIVATE INSTEAD OF PUBLIC, IT GAVE THE USAGES BUT IT STILL DIDN'T WORK....

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public DcMotor m_turnTableMotor;
    public DcMotor m_parkMotor;
    public DcMotor m_intakeMotor;
    public DcMotorEx m_launcherMotorRed;

    public CRServo m_intakeServo;
    public Servo m_hoodServo;
    public CRServo m_transferServo;
    public CRServo m_sorterServo;
    private Servo m_lightA;
    private Servo m_lightB;
    private Servo m_AxeServo;

    /* Sensors */
    public NormalizedColorSensor m_colorSensor;
    private RevTouchSensor m_limitSwitch;
    private Limelight3A m_limelight;



    /* Subsystems */
    private DriveSubsystem m_driveSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private IntakeMotorSubsystem m_intakeMotorSubsystem;
    private LimeLightSubsystem m_limelightSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;
    private AxeSubsystem m_axeSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private LauncherSubsystem m_launcherMotorSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ParkingSubsystem m_parkingSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private GamepadSubsystem m_gamepadSubsystem;
    private LightASubsystem m_lightASubsystem;
    private LightBSubsystem m_lightBSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;

    /* Commands */
    private DriveFieldOrientedCommand m_driveFieldOrientedCommand;
    private BellyOfTheBeastCommand m_bellyOfTheBeastCommand;
    private ResetTurnTableCommand m_resetTurnTableCommand;
    private IntakeMotorCommand m_intakeMotorCommand;
    private IntakeServoCommand m_intakeServoCommand;
    private OutTakeMotorCommand m_outTakeMotorCommand;
    private ColorModeOnCommand m_colorOnCommand;
    private TransferLimitCommand m_transferLimitCommand;
    private ReverseTransferCommand m_reverseTransferCommand;
    private Telemetry m_telemetry;


    private FudgeParkingCommand m_fudgeParkingCommand;
    private FudgeDeParkingCommand m_fudgeDeParkingCommand;
    private ParkingCommand m_parkingCommand;
    private DeParkingCommand m_deParkingCommand;
    private AimingOnCommand m_aimingCommand;
    private LauncherBellyCommand m_launcherBellyCommand;
    private ResetGyroCommand m_resetGyroCommand;
    private HoodUpCommand m_hoodFarCommand;
    private UnJamCommand m_unJamCommand;
    private GoBildaPinpointDriver m_odo;
    private TimerCommand m_timerCommand;

    /* PID */
    private PIDController m_pIDController;

    @Override
    public void initialize()
    {
        this.m_telemetry = telemetry;
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
        this.m_odo.setHeading(odoHeading, AngleUnit.DEGREES);
        double headingDegrees = currentPose.getHeading(UnnormalizedAngleUnit.DEGREES);


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
        this.m_intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.m_launcherMotorRed = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_parkMotor = hardwareMap.get(DcMotor.class, "parkMotor");

        this.m_lightA = hardwareMap.get(Servo.class, "lightA");
        this.m_lightB = hardwareMap.get(Servo.class, "lightB");


        /* Sensors */
        this.m_pIDController = new PIDController(0, 0, 0);
        this.m_pIDController.setPID(0.0, 0.0, 0.0);
        this.m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
        this.m_lightASubsystem = new LightASubsystem(hardwareMap, "lightA");
        this.m_lightBSubsystem = new LightBSubsystem(hardwareMap, "lightB");


        /* Subsystems */
        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2); //
        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_intakeMotorSubsystem = new IntakeMotorSubsystem(this.m_intakeMotor);
        this.m_launcherMotorSubsystem = new LauncherSubsystem(this.m_launcherMotorRed);
        this.m_parkingSubsystem = new ParkingSubsystem(this.m_parkMotor);
        this.m_limelightSubsystem = new LimeLightSubsystem(hardwareMap, 20);
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

        this.m_timerCommand = new TimerCommand (this.m_gamepadSubsystem, () -> getRuntime());
        this.m_gamepadSubsystem.setDefaultCommand(this.m_timerCommand);

        new Trigger(() -> timer.seconds() >= 95.0)
                .whenActive(new InstantCommand(() -> m_lightASubsystem.on(0.6), m_lightASubsystem));

        schedule();

        //DRIVER
        this.m_intakeMotorCommand = new IntakeMotorCommand(this.m_intakeMotorSubsystem);
        this.m_square = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenPressed(this.m_intakeMotorCommand);

        this.m_outTakeMotorCommand = new OutTakeMotorCommand(this.m_intakeMotorSubsystem);
        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenPressed(this.m_outTakeMotorCommand);

        this.m_resetGyroCommand = new ResetGyroCommand(this.m_gyroSubsystem);
        this.m_gyroResetButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.START))
                .whenPressed(this.m_resetGyroCommand);

        this.m_fudgeParkingCommand = new FudgeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadLeft = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_LEFT))
                .whileHeld(this.m_fudgeParkingCommand);

        this.m_fudgeDeParkingCommand = new FudgeDeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadRight = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_RIGHT))
                .whileHeld(this.m_fudgeDeParkingCommand);

        this.m_parkingCommand = new ParkingCommand(this.m_parkingSubsystem, this.m_intakeMotorSubsystem, this.m_launcherMotorSubsystem, this.m_sorterServoSubsystem, this.m_turnTableSubsystem, this.m_intakeServoSubsystem);
        this.m_dpadTop = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_UP))
                .whenPressed(this.m_parkingCommand);

        this.m_deParkingCommand = new DeParkingCommand(this.m_parkingSubsystem);
        this.m_dpadBottom = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(this.m_deParkingCommand);


        //GADGETEER
        this.m_colorOnCommand = new ColorModeOnCommand(this.m_colorSensorSubsystem, this.m_lightBSubsystem);
        this.m_leftBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenPressed(this.m_colorOnCommand);

        this.m_bellyOfTheBeastCommand = new BellyOfTheBeastCommand(this.m_sorterServoSubsystem);
        this.m_triangle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.Y))
                .toggleWhenPressed(this.m_bellyOfTheBeastCommand);

        this.m_launcherBellyCommand = new LauncherBellyCommand(this.m_launcherMotorSubsystem, this.m_axeSubsystem,this.m_hoodServoSubsystem, this.m_limelightSubsystem, this.m_sorterServoSubsystem);
        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
                .toggleWhenPressed(this.m_launcherBellyCommand);

        this.m_resetTurnTableCommand = new ResetTurnTableCommand(this.m_turnTableSubsystem);
        this.m_dpadLeft = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(this.m_resetTurnTableCommand);

        this.m_aimingCommand = new AimingOnCommand(this.m_limelightSubsystem, this.m_turnTableSubsystem, this.m_lightASubsystem, 20, this.m_telemetry);
        this.m_square = (new GamepadButton(this.m_driver2, GamepadKeys.Button.X))
                .toggleWhenPressed(this.m_aimingCommand);

        this.m_hoodFarCommand = new HoodUpCommand(this.m_hoodServoSubsystem);
        this.m_dpadTop = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_UP))
                .toggleWhenPressed(this.m_hoodFarCommand);

        this.m_reverseTransferCommand = new ReverseTransferCommand(this.m_limitSwitchSubsystem, () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        this.m_leftTrigger = new Trigger(() -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05);
        m_leftTrigger.whileActiveContinuous(m_reverseTransferCommand);

        this.m_transferLimitCommand = new TransferLimitCommand(this.m_limitSwitchSubsystem);
        this.m_rightBumper = (new GamepadButton(this.m_driver2, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_transferLimitCommand);

        this.m_unJamCommand = new UnJamCommand(this.m_sorterServoSubsystem, this.m_axeSubsystem, this.m_limitSwitchSubsystem);
        this.m_dpadRight = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_RIGHT))
                .whileHeld(this.m_unJamCommand);

        this.m_intakeServoCommand = new IntakeServoCommand(this.m_intakeServoSubsystem);
        m_shareButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.BACK))
                .whileHeld(this.m_intakeServoCommand);
    }
    @Override
    public void run()
    {
        // 1. This runs all your scheduled commands (Drive, Turntable, etc.)
        super.run();
        // 2. This pushes your Limelight data to the Driver Hub screen
        telemetry.update();
    }
}
