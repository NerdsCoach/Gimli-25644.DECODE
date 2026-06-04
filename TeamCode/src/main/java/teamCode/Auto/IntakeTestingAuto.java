package teamCode.Auto;


import static teamCode.Constants.AxeConstants.kAxeUp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.Constants;
import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.AimingOnCommand;
import teamCode.commands.AutoHoodCommand;
import teamCode.commands.AutoLauncherCommand;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferLimitCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightASubsystem;
import teamCode.subsystems.LightBSubsystem;
import teamCode.subsystems.LimeLightSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

@Autonomous(name="Testing Auto", group="Pinpoint")
//@Disabled

public class IntakeTestingAuto extends LinearOpMode
{
    /* Drivetrain */
    private MecanumDrive m_drive;

    /* PID */
    private PIDController m_pIDController;
    //Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx m_launcherMotor;
    public DcMotor m_turnTableMotor;
    public DcMotor m_intakeMotor;

    //Servos
    private Servo m_AxeServo;
    private CRServo m_transferServo;
    private CRServo m_sorterServo;
    private Servo m_hoodServo;

    //Sensors
    private GoBildaPinpointDriver m_odo;
    public NormalizedColorSensor m_colorSensor;
    private ElapsedTime m_StateTime = new ElapsedTime();
    private RevTouchSensor m_limitSwitch;

    //Subsystems
    private AxeSubsystem m_axeSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LightASubsystem m_lightASubsystem;
    private LightBSubsystem m_lightBSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private IntakeMotorSubsystem m_intakeMotorSubsystem;
    private LimeLightSubsystem m_limeLightSubsystem;

    //Commands
    private TransferLimitCommand m_transferLimitCommand;
    private AutoLauncherCommand m_autoLauncherCommand;
    private AutoHoodCommand m_autoHoodCommand;

    private TimerCommand m_timerCommand;
    private StateMachine m_stateMachine;

    private boolean lastState = false;
    public int count = 0;

    //Driving
    private final ElapsedTime holdTimer = new ElapsedTime();
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

        static final Pose2DUnNormalized Home = new Pose2DUnNormalized(DistanceUnit.MM, -360, -25, UnnormalizedAngleUnit.DEGREES, 0);


//    private static final double m_hoodDown = Constants.AimingConstants.kCloseAim;

    enum StateMachine
    {
        WAITING_FOR_START,
        PREPARE_FOR_BATTLE,
        START_PICK_UP,
        PICK_UP,
        PARKED,
        HOME
    }
    int ballCount = 0;
    int maxBalls = 9;//9

    private boolean m_aimingCommandStarted = false;
    private ElapsedTime m_aimingTimer = new ElapsedTime();


    private static final double m_axeUp = kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;
    private double m_lastKnownSpeed;
    ;

    @Override
    public void runOpMode()
    {
        // Initialize the hardware

        //Driving Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        //PinPoint
        m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        m_odo.setOffsets(-105, -25, DistanceUnit.MM);//these are tuned for Gimli 3110-0002-0001 Product Insight #1
        m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        m_odo.resetPosAndIMU();

        nav.setDriveType(DriveToPoint.DriveType.MECANUM);


        this.m_stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", m_odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", m_odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", m_odo.getDeviceVersion());
        telemetry.addData("Device Scalar", m_odo.getYawScalar());
        telemetry.update();



        //Mechanism Motors
        this.m_intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");        //Servos and Sensors
        this.m_launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        //Servos
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "aimingServo");
//        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");
        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");
        //Sensors
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        this.m_lightASubsystem = new LightASubsystem(hardwareMap, "lightA");
        this.m_lightBSubsystem = new LightBSubsystem(hardwareMap, "lightB");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
        this.m_limeLightSubsystem = new LimeLightSubsystem(hardwareMap, 20);

        //Subsystems
        this.m_intakeMotorSubsystem = new IntakeMotorSubsystem(this.m_intakeMotor);
//        this.m_intakeServoSybsystem = new IntakeServoSubsystem(this.m_intakeServo);
        this.m_launcherSubsystem = new LauncherSubsystem(this.m_launcherMotor);
        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
        this.m_limitSwitchSubsystem = new LimitSwitchSubsystem(this.m_limitSwitch, this.m_transferServo);
        //Commands
//        this.m_transferLimitCommand = new TransferLimitCommand(this.m_limitSwitchSubsystem);
        this.m_autoLauncherCommand = new AutoLauncherCommand(m_launcherSubsystem, m_axeSubsystem, m_hoodServoSubsystem, m_limeLightSubsystem);
        this.m_autoHoodCommand = new AutoHoodCommand(m_hoodServoSubsystem, this.m_limeLightSubsystem);

//        this.m_turnTableSubsystem.Turn(0);

        CommandScheduler.getInstance().reset();

        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            m_odo.update();

            switch (m_stateMachine)
            {
                case WAITING_FOR_START:
                    this.m_limitSwitchSubsystem.setTransferPower(0.0);
                    holdTimer.reset();
                    m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;

                    break;

                case PREPARE_FOR_BATTLE:
                    {
                        nav.resetPIDs();
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        m_autoLauncherCommand.schedule();
                        m_autoHoodCommand.schedule();
                        holdTimer.reset();

                        telemetry.addLine("Ready to Aim!");
                        m_stateMachine = StateMachine.HOME;
                    }
                    break;

                case HOME:

//                  if (!m_aimingCommandStarted)
                    {
                        telemetry.addLine("Aiming");
                        // Stop the drivetrain from the previous state
                        leftBack.setPower(0.0);
                        leftFront.setPower(0.0);
                        rightBack.setPower(0.0);
                        rightFront.setPower(0.0);

                        // Schedule AimingOnCommand with a timeout to ensure it finishes.
                        new AimingOnCommand(m_limeLightSubsystem, m_turnTableSubsystem, m_lightASubsystem, 20, telemetry)
                                .withTimeout(1500)
                                .schedule();

                        // Schedule LauncherOnCommand separately, so it continues to run after aiming is complete.
//                        m_autoLauncherCommand.schedule();
                        this.m_autoLauncherCommand.execute();


                        m_aimingTimer.reset();
                        m_aimingCommandStarted = true;
                    }
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", m_stateMachine);
            telemetry.addData("X coordinate (MM)", m_odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Y coordinate (MM)", m_odo.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading angle (DEGREES)", m_odo.getHeading(AngleUnit.DEGREES));


            telemetry.update();

        }
    }
}
