package teamCode;

import static teamCode.PoseStorage.currentPose;
import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.commands.HoodTestingDownCommand;
import teamCode.commands.HoodTestingUpCommand;
import teamCode.commands.AutoHoodCommand;
import teamCode.subsystems.GamepadSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LimeLightSubsystem;
//@Disabled

@TeleOp(name = "Hood Testing")
public class HoodTestingRobotContainer extends CommandOpMode
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
    private Button m_dpadTop2;
    private Button m_dpadBottom;
    private Button m_dpadLeft;
    private Button m_dpadRight;
    private Button m_gyroResetButton;
    private Button m_manateeButton;
    private Button m_scoreButton;
    private Button m_axeButton;
    private Button m_leftJoyStick;
    private Button m_rightTrigger;
    private Button m_optionsButton;
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
    private DcMotor m_intakeMotor;

    public Servo m_hoodServo;
    private CRServo m_intakeServo;
    public CRServo m_transferServo;
    public CRServo m_sorterServo;
    private Servo m_lightA;
    private Servo m_lightB;
    private Servo m_AxeServo;

    /* Sensors */
    public NormalizedColorSensor m_colorSensor;
    private RevTouchSensor m_limitSwitch;
    private RevTouchSensor m_intakeLimitSwitch;


    /* Subsystems */

    private HoodServoSubsystem m_hoodServoSubsystem;
    private GamepadSubsystem m_gamepadSubsystem;
    private LimeLightSubsystem m_limelightSubsystem;


    private Telemetry m_telemetry;
    private HoodTestingUpCommand m_hoodFudgeUpCommand;
    private HoodTestingDownCommand m_hoodFudgeDownCommand;

    private GoBildaPinpointDriver m_odo;

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

        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");
        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");

        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        this.m_launcherMotorRed = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_parkMotor = hardwareMap.get(DcMotor.class, "parkMotor");
        this.m_intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.m_lightA = hardwareMap.get(Servo.class, "lightA");
        this.m_lightB = hardwareMap.get(Servo.class, "lightB");

        /* Sensors */
        this.m_pIDController = new PIDController(0, 0, 0);
        this.m_pIDController.setPID(0.0, 0.0, 0.0);
        this.m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
        this.m_intakeLimitSwitch = hardwareMap.get(RevTouchSensor.class, "intakeLimitSwitch");

        /* Subsystems */
        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2); // TODO added light
        this.m_limelightSubsystem = new LimeLightSubsystem(hardwareMap, 20);
        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "aimingServo");

        register(this.m_gamepadSubsystem);

        schedule();
        //DRIVER

        //DPAD UP, DOWN, LEFT, RIGHT

        this.m_hoodFudgeUpCommand = new HoodTestingUpCommand(this.m_hoodServoSubsystem);
        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(this.m_hoodFudgeUpCommand);

        this.m_hoodFudgeDownCommand = new HoodTestingDownCommand(this.m_hoodServoSubsystem);
        this.m_rightBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_hoodFudgeDownCommand);

//        this.m_aLaunch = new ALauncherOnCommand(this.m_launcherMotorSubsystem, this.m_axeSubsystem);
//        this.m_dpadTop = (new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_UP))
//                .whenPressed(this.m_aLaunch);

    }

    @Override
    public void run()
    {
        super.run();
        telemetry.addData("--- HOOD TUNING MODE ---", "");
        telemetry.addData("Current Servo Position Value", this.m_hoodServoSubsystem.getTuningPosition());
        telemetry.update();
//        m_telemetry.addData("Current Velocity", this.m_launcherMotorSubsystem.getVel());
    }
}
