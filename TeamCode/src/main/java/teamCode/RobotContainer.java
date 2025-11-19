package teamCode;

import static teamCode.PoseStorage.currentPose;
import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import android.provider.Settings;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.commands.AimingCommand;
import teamCode.commands.DriveFieldOrientedCommand;
import teamCode.commands.DriveManateeModeCommand;
import teamCode.commands.IntakeModeCommand;
import teamCode.commands.IntakeResetCommand;
import teamCode.commands.IntakeWheelCommand;
import teamCode.commands.LauncherCommand;
import teamCode.commands.ParkingCommand;
import teamCode.commands.ResetGyroCommand;
import teamCode.commands.ScoreModeCommand;
//import teamCode.commands.ScoreTestingCommand;
import teamCode.commands.SorterServoCommand;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferServoCommand;
import teamCode.commands.TurnTableCommand;

import teamCode.subsystems.AimingServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.GimlisBoxMotorSubsystem;
import teamCode.subsystems.HuskyLensAimingSubsystem;
import teamCode.subsystems.GamepadSubsystem;
import teamCode.subsystems.TransferServoSubsystem;
import teamCode.subsystems.GyroSubsystem;
import teamCode.subsystems.LauncherMotorSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;

@TeleOp(name = "DECODE")
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

    /* Motors */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor m_turnTableMotor;
    public DcMotor m_parkingMotor;
    public DcMotor m_launcherMotorRed;
    public DcMotor m_launcherMotorBlue;


    public CRServo m_intakeServo;
    public Servo m_aimingServo;
    public Servo m_transferServo;
    public Servo m_sorterServo;

    /* Sensors */
    public HuskyLens m_huskylens;
    public ColorSensor m_colorSensor;

    /* Subsystems */
    private DriveSubsystem m_driveSubsystem;
    private AimingServoSubsystem m_aimingSubsystem;
    private TurnTableSubsystem m_turnTableMotorSubsystem;
    private HuskyLensAimingSubsystem m_huskyLensSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;
    private TransferServoSubsystem m_transferServoSubsystem;
    private LauncherMotorSubsystem m_launcherMotorSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private GimlisBoxMotorSubsystem m_parkingSubsystem;
    private GyroSubsystem m_gyroSubsystem;
    private GamepadSubsystem m_gamepadSubsystem;


    /* Commands */
    private DriveFieldOrientedCommand m_driveFieldOrientedCommand;
    private IntakeWheelCommand m_intakeWheelCommand;
    private IntakeResetCommand m_intakeResetCommand;
    private LauncherCommand m_launcherCommand;
    private TurnTableCommand m_turnTableCommand;
    private SorterServoCommand m_sorterServoCommand;
    private TransferServoCommand m_transferServoCommand;

    private ParkingCommand m_parkingCommand;
    private AimingCommand m_aimingCommand;
    private DriveManateeModeCommand m_driveManateeModeCommand;
    private ResetGyroCommand m_resetGyroCommand;

    private GoBildaPinpointDriver m_odo;
    private TimerCommand m_timerCommand;

    private IntakeModeCommand m_intakeMode;
    private ScoreModeCommand m_scoreMode;

//    private ScoreTestingCommand m_scoreMode;

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

        /* Gamepad */

        this.m_driver1 = new GamepadEx(gamepad1);
        this.m_driver2 = new GamepadEx(gamepad2);

        /* Motors */
    //TODO Device Name MUST MATCH name on the Drivers Station!!!!!

        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");
        this.m_sorterServo = hardwareMap.get(ServoImplEx.class, "sorterServo");
        this.m_transferServo = hardwareMap.get(Servo.class, "transferServo");


        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        new Motor(hardwareMap, "turnTableMotor", Motor.GoBILDA.RPM_435);

        this.m_launcherMotorRed = hardwareMap.get(DcMotor.class, "launcherMotorRed");
        this.m_launcherMotorBlue = hardwareMap.get(DcMotor.class, "launcherMotorBlue");
//        this.m_parkingMotor = hardwareMap.get(DcMotor.class, "parkingMotor");


        /* Sensors */
        this.m_pIDController = new PIDController(0, 0, 0);
        this.m_pIDController.setPID(0.0, 0.0, 0.0);
//        this.m_huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        //TODO Color Sensor Here:

        /* Subsystems */

        this.m_driveSubsystem = new DriveSubsystem(this.m_drive, this.m_odo/*, this.m_goBilda*/);
        this.m_gyroSubsystem = new GyroSubsystem(this.m_odo);
        this.m_gamepadSubsystem = new GamepadSubsystem(this.m_driver1, this.m_driver2);
        this.m_turnTableMotorSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_launcherMotorSubsystem = new LauncherMotorSubsystem(this.m_launcherMotorRed,this.m_launcherMotorBlue);

        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);
        this.m_sorterServoSubsystem = new SorterServoSubsystem(hardwareMap, "sorterServo");
        this.m_transferServoSubsystem = new TransferServoSubsystem(hardwareMap, "transferServo");


        register(this.m_driveSubsystem);
        register(this.m_intakeServoSubsystem);

        /* Default Commands */

        this.m_driveFieldOrientedCommand = new DriveFieldOrientedCommand
                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
//
//        this.m_driveManateeModeCommand = new DriveManateeModeCommand
//                (this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
//                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());

        this.m_driveSubsystem.setDefaultCommand(this.m_driveFieldOrientedCommand);

        this.m_timerCommand = new TimerCommand (this.m_gamepadSubsystem, () -> getRuntime());
        this.m_gamepadSubsystem.setDefaultCommand(this.m_timerCommand);

        this.m_intakeWheelCommand = new IntakeWheelCommand(this.m_intakeServoSubsystem, () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> this.m_driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        this.m_intakeServoSubsystem.setDefaultCommand(this.m_intakeWheelCommand);


//        this.m_scoreMode = new ScoreTestingCommand(this.m_sorterServoSubsystem, this.m_transferServoSubsystem, this.m_launcherMotorSubsystem);
//        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
//                .whenPressed(this.m_scoreMode);

//        this.m_scoreMode = new ScoreModeCommand(this.m_sorterServoSubsystem, this.m_transferServoSubsystem, this.m_launcherMotorSubsystem);
//        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
//                .whenPressed(this.m_scoreMode);
        schedule();
        /* Event Commands */
        this.m_scoreMode = new ScoreModeCommand(this.m_sorterServoSubsystem, this.m_transferServoSubsystem, this.m_launcherMotorSubsystem);
        this.m_xButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.A))
                .whenPressed(this.m_scoreMode);

        this.m_intakeMode = new IntakeModeCommand(this.m_sorterServoSubsystem);
        this.m_dpadBottom = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(this.m_intakeMode);

        this.m_intakeResetCommand = new IntakeResetCommand(this.m_sorterServoSubsystem);
        this.m_dpadTop = (new GamepadButton(this.m_driver2, GamepadKeys.Button.DPAD_UP))
                .whenPressed(this.m_intakeResetCommand);
//        this.m_driveManateeModeCommand = new DriveManateeModeCommand(this.m_driveSubsystem,this.m_gamepadSubsystem,() -> this.m_driver1.getLeftX(),
//                () -> this.m_driver1.getLeftY(), () -> this.m_driver1.getRightX(), () -> this.m_driver1.getRightY());
//        this.m_manateeButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.B))
//                .toggleWhenPressed(this.m_driveFieldOrientedCommand, this.m_driveManateeModeCommand);

//        this.m_scoreButton = (new GamepadButton(this.m_driver2, GamepadKeys.Button.B))
//                .toggleWhenPressed(this.m_intakeMode, this.m_scoreMode);

//        this.m_aimingCommand = new AimingCommand(this.m_huskyLensSubsystem, this.m_aimingSubsystem);
//        this.m_leftBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.LEFT_BUMPER))
//                .whenPressed(this.m_aimingCommand);

//        this.m_parkingCommand = new ParkingCommand(this.m_parkingSubsystem);
//        this.m_square = (new GamepadButton(this.m_driver1, GamepadKeys.Button.X))
//                .whenPressed(this.m_parkingCommand);
        this.m_launcherCommand = new LauncherCommand(this.m_launcherMotorSubsystem);
        this.m_rightBumper = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_launcherCommand);

//        this.m_sorterServoCommand = new SorterServoCommand(this.m_sorterServoSubsystem);
//        this.m_circle = (new GamepadButton(this.m_driver1, GamepadKeys.Button.B))
//                .whenPressed(this.m_sorterServoCommand);
        this.m_transferServoCommand = new TransferServoCommand(this.m_transferServoSubsystem);
        this.m_triangle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.Y))
                .whenPressed(this.m_transferServoCommand);


        this.m_resetGyroCommand = new ResetGyroCommand(this.m_gyroSubsystem);
        this.m_gyroResetButton = (new GamepadButton(this.m_driver1, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(this.m_resetGyroCommand);
//
//
//        this.m_pose2DObservationZoneCommand = new Pose2DObservationZoneCommand(this.m_driveSubsystem, this.m_odo,
//                this.leftFront, this.rightFront, this.leftBack,this.rightBack);
//        new GamepadButton(this.m_driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(this.m_pose2DObservationZoneCommand);
    }
//  {
//        for (int i = 1; i>0; i+=0)
//        {
//            telemetry.addData("Lift Arm", this.m_liftArmMotor.getCurrentPosition());
//            telemetry.addData("Slide Arm", this.m_slideArmMotor.getCurrentPosition());
//            telemetry.update();
//        }

//    }
//{
//        for (int i = 1; i>0; i+=0)
//        {
//           telemetry.addData("SorterServo", this.m_sorterServo.
//            telemetry.update();
//        }

//    }
}
