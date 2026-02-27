package teamCode.Auto;


import static teamCode.Constants.AxeConstants.kAxeDown;
import static teamCode.Constants.AxeConstants.kAxeDownAuto;
import static teamCode.Constants.AxeConstants.kAxeUp;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.Constants;
import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.AimingOnCommand;
import teamCode.commands.LauncherOnCommand;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferLimitCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.LimeLightSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

@Autonomous(name="Blue Wall Auto", group="Pinpoint")
//@Disabled

public class BlueWallAuto extends LinearOpMode
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
    private LightSubsystem m_lightSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private IntakeMotorSubsystem m_intakeMotorSubsystem;
    private LimeLightSubsystem m_limeLightSubsystem;

    //Commands
    private TransferLimitCommand m_transferLimitCommand;
    private LauncherOnCommand m_launcherOnCommand;

    private TimerCommand m_timerCommand;
    private StateMachine m_stateMachine;

    private boolean lastState = false;
    public int count = 0;


    //Driving
    private final ElapsedTime holdTimer = new ElapsedTime();
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    static final Pose2DUnNormalized Start = new Pose2DUnNormalized(DistanceUnit.MM, 0, 0, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Move = new Pose2DUnNormalized(DistanceUnit.MM, 50, -255, UnnormalizedAngleUnit.DEGREES, 0); //-275
    static final Pose2DUnNormalized StartPickUp = new Pose2DUnNormalized(DistanceUnit.MM, 320, -700, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized EndPickUp = new Pose2DUnNormalized(DistanceUnit.MM, 1250, -710, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Park = new Pose2DUnNormalized(DistanceUnit.MM, 400, -200, UnnormalizedAngleUnit.DEGREES, 0);

    private static final double m_aimFar = Constants.AimingConstants.kFarAim;
    private static final double m_hoodDown = Constants.AimingConstants.kCloseAim;

    private String[] targetPattern = {"NONE", "NONE", "NONE"};
    private int ballsScored = 0;

    enum StateMachine
    {
        WAITING_FOR_START,
        SCAN_OBELISK,
        GPP_PATTERN,
        PGP_PATTERN,
        PPG_PATTERN,
        PREPARE_FOR_BATTLE,
        LAUNCH_BALL_1,
        WAIT_1,
        COUNTER_SCORE,
        LAUNCH_BALL_2,
        WAIT_2,
        LAUNCH_BALL_3,
        WAIT_3,
        LAUNCH_BALL_4,
        WAIT_4,
        START_PICK_UP,
        PICK_UP,
        PARKED, WAIT_FOR_NEXT, REVERSE_RECOVERY, LAUNCH_BALL, END, AIM_TURNTABLE, POWER_LAUNCHER, SORT_PURPLE_ONE, SORT_2,
    }
    int ballCount = 0;
    int maxBalls = 7;

    int tagPattern = 0;

    private boolean m_aimingCommandStarted = false;
    private ElapsedTime m_aimingTimer = new ElapsedTime();

    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;

    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;
    private double m_lastKnownSpeed;

    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public double m_lastKnownColor;
    private static final int  m_off = 1;
    private static final int  m_on = 0;





    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

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

        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");

        //Mechanism Motors
        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        this.m_launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");        //Servos and Sensors
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "aimingServo");
        this.m_limeLightSubsystem = new LimeLightSubsystem(hardwareMap, 20);

        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);

        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_limitSwitchSubsystem = new LimitSwitchSubsystem(this.m_limitSwitch, this.m_transferServo);
        this.m_transferLimitCommand = new TransferLimitCommand(this.m_limitSwitchSubsystem);
        this.m_intakeMotorSubsystem = new IntakeMotorSubsystem(this.m_intakeMotor);

        this.m_launcherSubsystem = new LauncherSubsystem(this.m_launcherMotor);
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        this.m_launcherOnCommand = new LauncherOnCommand(m_launcherSubsystem, m_axeSubsystem, m_hoodServoSubsystem, m_limeLightSubsystem);

        CommandScheduler.getInstance().reset();

        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            m_odo.update();
            NormalizedRGBA colors = m_colorSensorSubsystem.getNormalizedColors();

            float[] hsv = new float[3];
            Color.RGBToHSV(
                    (int) (colors.red * 255),
                    (int) (colors.green * 255),
                    (int) (colors.blue * 255),
                    hsv
            );
            float hue = hsv[0];

            switch (m_stateMachine)
            {
                //TODO: Start the robot with the launcher aimed at the obelisk
                case WAITING_FOR_START:
                    holdTimer.reset();

                    m_stateMachine = StateMachine.SCAN_OBELISK;
                    break;

                //TODO: Scan Obelisk and detect tag number
                //TODO: Go to correct pattern case
                //TODO: If 21(GPP)
                // {
                //      if (it see's green)
                //      {
                //          put axe down
                //          and go to launch
                //      }
                //      if (see's purple)
                // }

                //TODO: If 22(PGP)
                // {
                // if(it see's green and then purple)
                // {
                //      put axe down and go to launch
                // }
                // }


                //Todo: If 23(PPG)


                case SCAN_OBELISK:
                    LLResult result = m_limeLightSubsystem.getLatestResult();

                    // Check if we actually see any tags
                    if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

                        // Loop through all seen tags to find the Obelisk
                        for (LLResultTypes.FiducialResult fr : result.getFiducialResults())
                        {
                            int id = fr.getFiducialId();

                            if (id == 21)
                            {
                                targetPattern = new String[]{"GREEN", "PURPLE", "PURPLE"};
                                telemetry.addLine("GPP");
                                this.m_axeSubsystem.pivotAxe(kAxeUp);
                                this.tagPattern = 21;
                                m_stateMachine = StateMachine.GPP_PATTERN;
                            }
                            else if (id == 22)
                            {
                                targetPattern = new String[]{"PURPLE", "GREEN", "PURPLE"};
                                telemetry.addLine("PGP");
                                this.tagPattern = 22;
                                holdTimer.reset();
                                m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;

                            }
                            else if (id == 23)
                            {
                                targetPattern = new String[]{"PURPLE", "PURPLE", "GREEN"};
                                telemetry.addLine("PPG");
                                this.m_axeSubsystem.pivotAxe(kAxeUp);
                                this.tagPattern = 23;
                                m_stateMachine = StateMachine.PPG_PATTERN;
                            }
                        }
                    }
                    break;

                case PREPARE_FOR_BATTLE:
                    this.m_hoodServoSubsystem.pivotHood(m_aimFar);
                    this.m_axeSubsystem.pivotAxe(kAxeDown);
                    this.m_sorterServoSubsystem.spinSorter(-1.0);

                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            Move, 0.6, 0.1)|| holdTimer.seconds() >= 4.0)
                    {
                        nav.resetPIDs();
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);

                        holdTimer.reset();

                        telemetry.addLine("Launch Motor On");
                        m_stateMachine = StateMachine.AIM_TURNTABLE;
                    }
                    break;

                case GPP_PATTERN:
                    this.m_sorterServoSubsystem.spinSorter(-0.5);
                    this.m_axeSubsystem.pivotAxe(kAxeUp);
                    telemetry.addLine("GPP Color Sensing");
                    if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
                    {
                        this.m_lightSubsystem.setLEDGreen();
                        this.m_axeSubsystem.pivotAxe(kAxeDown);
                        holdTimer.reset();
                        m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;
                        this.m_lastKnownColor = 0.5;
                    }
                 break;

                case PPG_PATTERN:
                    this.m_sorterServoSubsystem.spinSorter(-0.3);
                    this.m_axeSubsystem.pivotAxe(kAxeUp);
                    telemetry.addLine("PPG Color Sensing");
                    holdTimer.reset();
                    if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
                    {
                        this.m_lightSubsystem.setLEDGreen();
                        this.m_lastKnownColor = 0.5;
                        m_stateMachine = StateMachine.SORT_PURPLE_ONE;
                    }
                    break;

                case PGP_PATTERN:
                    this.m_sorterServoSubsystem.spinSorter(-0.3);
                    this.m_axeSubsystem.pivotAxe(kAxeUp);
                    telemetry.addLine("PPG Color Sensing");
                    holdTimer.reset();
                    if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
                    {
                        this.m_lightSubsystem.setLEDPurple();
                        this.m_axeSubsystem.pivotAxe(kAxeDown);
                        this.m_lastKnownColor = 0.722;
                        holdTimer.reset();
                        m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;
                    }
                    break;

                case SORT_PURPLE_ONE:
                if(Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
                {
                    this.m_axeSubsystem.pivotAxe(kAxeDown);
                    m_lightSubsystem.setLEDPurple();
                    this.m_lastKnownColor = 0.722;
                    holdTimer.reset();
                    m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;
                }
                break;

                case AIM_TURNTABLE:
                    //TODO: Turn table to the Goal April tag
                    if (!m_aimingCommandStarted)
                    {
                        telemetry.addLine("Aiming");
                        // Stop the drivetrain from the previous state
                        leftBack.setPower(0.0);
                        leftFront.setPower(0.0);
                        rightBack.setPower(0.0);
                        rightFront.setPower(0.0);

                        // Schedule AimingOnCommand with a timeout to ensure it finishes.
                        new AimingOnCommand(m_limeLightSubsystem, m_turnTableSubsystem, m_lightSubsystem, 20, telemetry)
                                .withTimeout(3000)
                                .schedule();

                        // Schedule LauncherOnCommand separately, so it continues to run after aiming is complete.
                        m_launcherOnCommand.schedule();

                        m_aimingTimer.reset();
                        m_aimingCommandStarted = true;
                    }

                    // After 3 seconds, the aiming timeout will have triggered. Launch to the next state.
                    if (m_aimingTimer.seconds() > 3.0)
                    {
                        m_stateMachine = StateMachine.LAUNCH_BALL;
                    }
                    break;

                case LAUNCH_BALL:
                    // Standard launch power
                    this.m_limitSwitchSubsystem.setTransferPower(-1.0);
                    boolean currentState = m_limitSwitchSubsystem.isPressed();

                    if (currentState && !lastState)
                    {
                        // SUCCESS: Ball passed
                        m_limitSwitchSubsystem.setTransferPower(0.0);
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.WAIT_FOR_NEXT;
                    }
                    else if (m_StateTime.time() > 2.0)
                    {
                        // JAM DETECTED: Switch wasn't hit in 2 seconds
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.REVERSE_RECOVERY;
                    }
                    lastState = currentState;
                    break;

                case WAIT_FOR_NEXT:
                    if (m_StateTime.time() > 0.5)
                    {
                        ballCount++; // Increment after successful transfer

                        if (ballCount==3)
                        {
                            holdTimer.reset();
                            m_stateMachine = StateMachine.START_PICK_UP;

                        }
                        else if (ballCount==5)
                        {
                            holdTimer.reset();
                            m_StateTime.reset();
                            this.m_intakeMotorSubsystem.spinMotorIntake(0.5);
                            m_stateMachine = StateMachine.LAUNCH_BALL;
                        }
                        else if (ballCount < maxBalls && ballCount!=3 && ballCount!=5)
                        {
                            // Still have balls left: loop back to launch
                            m_StateTime.reset();
                            m_stateMachine = StateMachine.LAUNCH_BALL;
                        } else
                        {
                            // Done with all 4: reset counter and move on
                            ballCount = 0;
                            m_stateMachine = StateMachine.PARKED;
                            holdTimer.reset();
                        }
                    }
                    break;

                case REVERSE_RECOVERY:
                    this.m_limitSwitchSubsystem.setTransferPower(0.30); // Reverse
                    if (m_StateTime.time() > 0.3)
                    {
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.LAUNCH_BALL; // Retry the same ball
                    }
                    break;

                case START_PICK_UP:
                    // We are done launching, so cancel the launcher command.
                    m_launcherOnCommand.cancel();
                    this.m_sorterServoSubsystem.spinSorter(0.0);
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            StartPickUp, 0.4, .5)|| holdTimer.seconds() >= 3.0)
                    {
//                        this.m_axeSubsystem.pivotAxe(kAxeUp);
                        this.m_intakeMotorSubsystem.spinMotorIntake(0.5);
                        holdTimer.reset();
                        m_stateMachine = StateMachine.PICK_UP;
                        telemetry.addLine("Start Pick Up");
                    }
                    break;

                case PICK_UP:
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            EndPickUp, 0.2, 0.5) || holdTimer.seconds() >= 3.0)
                    {
                        m_aimingCommandStarted = false;
                        this.m_intakeMotorSubsystem.spinMotorIntake(0.0);
                        holdTimer.reset();
                        m_stateMachine = StateMachine.SORT_2;
                        telemetry.addLine("End Pickup");
                    }
                    break;

                case SORT_2:
                    if (tagPattern == 21)
                    {
                        targetPattern = new String[]{"GREEN", "PURPLE", "PURPLE"};
                        telemetry.addLine("GPP");
                        holdTimer.reset();
                        m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;
                    }
                    else if (tagPattern == 22)
                    {
                        targetPattern = new String[]{"PURPLE", "GREEN", "PURPLE"};
                        telemetry.addLine("PGP");

                        holdTimer.reset();
                        m_stateMachine = StateMachine.PGP_PATTERN;

                    }
                    else if (tagPattern == 23)
                    {
                        targetPattern = new String[]{"PURPLE", "PURPLE", "GREEN"};
                        telemetry.addLine("PPG");
                        this.m_axeSubsystem.pivotAxe(kAxeUp);

                        m_stateMachine = StateMachine.PPG_PATTERN;
                    }

                case PARKED:
                    // Make sure the launcher command is stopped.
                    m_launcherOnCommand.cancel();
                    this.m_turnTableMotor.setTargetPosition(-500);
                    this.m_axeSubsystem.pivotAxe(kAxeUp);
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            Park, 0.4, 0.5))
                    {
                        leftBack.setPower(0.0);
                        leftFront.setPower(0.0);
                        rightBack.setPower(0.0);
                        rightFront.setPower(0.0);
                        this.m_hoodServoSubsystem.pivotHood(m_aimFar);
                        this.m_limitSwitchSubsystem.setTransferPower(0.0);
                        this.m_sorterServoSubsystem.spinSorter(0.0);
                        this.m_intakeMotorSubsystem.stop();
                        this.m_launcherSubsystem.setMotorVelocity(0);

                        telemetry.addLine("Done");
                    }
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", m_stateMachine);
            telemetry.addData("X coordinate (MM)", m_odo.getEncoderX());
            telemetry.addData("Y coordinate (MM)", m_odo.getEncoderY());
            telemetry.addData("Heading angle (DEGREES)", m_odo.getHeading(AngleUnit.DEGREES));

            telemetry.update();

        }
    }
}   // end class

