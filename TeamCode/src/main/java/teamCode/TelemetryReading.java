package teamCode;

import static teamCode.PoseStorage.currentPose;
import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.commands.TimerCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.GamepadSubsystem;
import teamCode.subsystems.GyroSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.ParkingSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

@TeleOp(name = "Read Telemetry")
public class TelemetryReading extends CommandOpMode
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

    /* Motors */
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

//    public int m_gain;
//    public  m_hsvValues;

    /* Subsystems */
    private DriveSubsystem m_driveSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private HuskyLensSubsystem m_huskyLensSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;
    private AxeSubsystem m_axeSubsystem;
    private LauncherSubsystem m_launcherMotorSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ParkingSubsystem m_parkingSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private GamepadSubsystem m_gamepadSubsystem;
    private LightSubsystem m_lightSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;

    private GoBildaPinpointDriver m_odo;
    private TimerCommand m_timerCommand;

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

        /* Subsystems */

        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2);
        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_launcherMotorSubsystem = new LauncherSubsystem(this.m_launcherMotorRed);
        this.m_parkingSubsystem = new ParkingSubsystem(this.m_parkMotor);
        this.m_huskyLensSubsystem = new HuskyLensSubsystem(this.m_huskylens, 4);
        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);
        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
//        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "hoodServo");


        register(this.m_driveSubsystem);
        register(this.m_intakeServoSubsystem);

        schedule();

//    {
//        telemetry.addLine()
//                .addData("Red", "%.3f", m_colorSubsystem.getRed())
//                .addData("Green", "%.3f", m_colorSubsystem.getRed())
//                .addData("Blue", "%.3f", m_colorSubsystem.getRed());
//        telemetry.addLine()
//                .addData("Hue", "%.3f", m_hsvValues)
//                .addData("Saturation", "%.3f", m_hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", m_colorSensor.alpha());
//    }

        for (int i = 1; i>0; i+=0)
        {
            telemetry.addData("Turn Table Motor", this.m_turnTableMotor.getCurrentPosition());
            telemetry.addData("Parking Motor", this.m_parkMotor.getCurrentPosition());
            telemetry.addData("Launcher Motor", this.m_launcherMotorRed.getPower());
            telemetry.addData("Switch Pressed", this.m_limitSwitch.isPressed());
            System.out.println(this.m_limitSwitch.isPressed());
        }
            telemetry.update();
        }

//    }
}
