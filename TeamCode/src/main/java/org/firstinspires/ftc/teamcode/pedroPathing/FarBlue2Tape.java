package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPIDController;
import org.firstinspires.ftc.teamcode.GlobalOffsets;
import org.firstinspires.ftc.teamcode.StateVars;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Far Blue 2 Tape", group="Robot")
public class FarBlue2Tape extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout = 0;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, shoot1, shoot0, movePoint;
    private Pose[] pickup1 = new Pose[2];
    private Pose[] pickup2 = new Pose[2];
    private Pose[] pickup3 = new Pose[3];
    private Pose[] junoPose = new Pose[3];
    private PathChain scorePath0, scorePath1, scorePath2, scorePath3, moveScore,limelightPath,gatePath, pickupPath1, pickupPath2, pickupPath3;
    private PathChain[] junoPath = new PathChain[2];
    //endregion

    //region PARA CALLBACKS
    FakeParameticCallback[] current = new FakeParameticCallback[2];

    FakeParameticCallback pickupCall1, pickupCall2, pickupCall3, scoreCall0, scoreCall1, scoreCall2, scoreCall3, turretCall3;
    //endregion

    //region HARDWARE DECLARATIONS
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private DcMotor trans = null;

    //LIMELIGHT
    private Limelight3A limelight;

    //SERVOS
    private CRServo spin1 = null;
    private CRServo spin2 = null;
    private Servo led = null;
    private Servo hood = null;
    private CRServo turret1 = null;
    private CRServo turret2 = null;
    private Servo llservo = null;
//    private Servo tempServo = null;

    // ENCODERS
    private GoBildaPinpointDriver pinpoint = null;
    private AnalogInput spinEncoder;
    private AnalogInput turretEncoder;

    //COLOR
    private NormalizedColorSensor color1 = null;
    private NormalizedColorSensor color2 = null;
    //endregion

    //region VISION SYSTEM
    // AprilTag Configuration
    private boolean createLimelightPathOn = false;
    private AprilTagDetection desiredTag;

    // FTC Vision Portal
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Tracking State
    private boolean facingGoal = false;
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;
    private double txOffset = 0;
    private double distCamOffset = 0;

    // Heading PID
    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_TURN = 0.4;
    private double KballAngle = 0.67287181376;
    private double Kball = 0.6846;
    private double ballYoffset = 35;
    //limelight path
    private double ballX;
    private double ballY;
    private double ballHeading;
    double limelightWallPos;
    //endregion

    //region SHOOTING SYSTEM
    private FlywheelPIDController flywheel;
    private double flySpeed = 0.0;
    private boolean shootReady = false;
    private boolean isInitialized = false;

    private static final double[] CAM_RANGE_SAMPLES =   {25, 31.8, 37, 39.2, 44.2,  52.6, 53.1, 56.9, 61.5, 65.6, 70.3, 73.4, 77.5, 84.3, 91.8, 100.4, 110.0, 118.4};
    private static final double[] ODOM_RANGE_SAMPLES =  {45.2, 50.2, 55.3, 60.9, 66.5, 72.2, 76.7, 81.1, 86.3, 90.9, 96.2, 99.7, 104.3, 109.9, 118.1, 128.5, 139.6, 148.7};
    private static final double[] FLY_SPEEDS =          {1004, 1016, 1041, 1071, 1115, 1132, 1143, 1151, 1212, 1236, 1244, 1252, 1253, 1259, 1273, 1358, 1387, 1421};
    private static final double[] AIR_TIME =   {2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 2.89, 3, 3.23, 3.5, 3.79, 4.27};  //seconds divide all by 4
    private static final double[] HOOD_ANGLES = GlobalOffsets.globalHoodAngles;
    private double smoothedRange = 0;
    private static final double ALPHA = 0.8;
    private boolean flyHoodLock = false;

    // Auto Shooting State
    private int autoShootNum = 3;
    private double autoShootTime = 0;
    private boolean autoShot = false;
    private double lastTriggered = 0;
    boolean isRapidFire = false;
    double rapidFireStartTime = 0;
    //endregion

    //region HOOD SYSTEM
    // Hood Positions
    private double hoodAngle = 138.3;
    private double hoodOffset = 0;
    //endregion

    //region SPINDEXER SYSTEM
    // Spindexer PIDF Constants
    private double pidKp = 0.004;
    private double pidKi = 0.001;
    private double pidKd = 0.00035;//0.00065
    private double pidKf = 0.022;

    // Spindexer PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 6.0;
    private double pidLastTimeMs = 0.0;
    private double lastFilteredD = 0.0;

    // Spindexer Control Parameters
    private final double positionToleranceDeg = 1.0;
    private final double outputDeadband = 0.03;
    private boolean spindexerOverride = false;
    private double overrideTime = 0.0;

    // Spindexer Positions
    private final double[] SPINDEXER_POSITIONS = {51.75, 81.75, 111.75, 141.75, 171.75, 21.75};
    private int spindexerIndex = 0;
    private int prevSpindexerIndex = 0;
    private int greenPos = 0;

    // Ball Storage Tracking
    private char[] savedBalls = {'g', 'p', 'p'};
    private boolean[] presentBalls = {false, false, false};

    private boolean spindexerPidArmed = false;
    private boolean spindexerAtTarget = false;
    private boolean intakeOn = false;
    //endregion

    //region TURRET SYSTEM
    // PIDF Constants
    private double tuKp = 0.0058;
    private double tuKi = 0.0006;
    private double tuKd = 0.00015;
    private double tuKf = 0.005;
    private static final double tuKv = 0.0001;

    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    // PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 90.0;
    private double tuLastD = 0.0;

    // Control Parameters
    private final double tuToleranceDeg = 0.85;
    private final double tuDeadband = 0.03;

    // Turret Position
    private double tuPos = 0.0;
    private static final double TURRET_LIMIT_DEG = 150.0;
    private double tuOffset = 0.0;
    private boolean trackingOn = true;
    //endregion

    //region VARIANT VARS (Alliance Specific)
    private static final double goalX = 0;
    private static final double goalY = 144;
    private static double turretClock = -1;//1 red, -1 blue

    //endregion
    double shotTime = 0;
    Vector velocity = new Vector(0,0);

    private final PathConstraints shootConstraints = new PathConstraints(0.99, 100, 0.85, 1);

    public void createPoses(){
        startPose = new Pose(56.8,8.5,Math.toRadians(90));

        //0 is control point, 1 is endpoint
        pickup1[0] = new Pose(60.86,40.52,Math.toRadians(180));
        pickup1[1] = new Pose(10,35.68,Math.toRadians(180));

        pickup2[0] = new Pose(64.0,68.6,Math.toRadians(180));
        pickup2[1] = new Pose(9,60.55,Math.toRadians(180));

        pickup3[0] = new Pose(15.72,24.91,Math.toRadians(180));
        pickup3[1] = new Pose(8.81,10.39,Math.toRadians(220));

        junoPose[0] = new Pose(29.78,15.40,Math.toRadians(180));//backup pose
        junoPose[1] = new Pose(24.03,4.75,Math.toRadians(180));//control point
        junoPose[2] = new Pose(8.59,5.09,Math.toRadians(180));

        shoot0 = new Pose(62.5,26.5,Math.toRadians(180));
        shoot1 = new Pose(58,19,Math.toRadians(180));
        movePoint = new Pose(35.5,18.5,Math.toRadians(90));
    }

    public void createPaths(){
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot0))
                .setConstraints(shootConstraints)
                .setLinearHeadingInterpolation(startPose.getHeading(),shoot0.getHeading())
                .build();
        scoreCall0 = new FakeParameticCallback(0.8,()-> shootReady=true,follower);
        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot0,pickup1[0],pickup1[1]))
                .setConstantHeadingInterpolation(shoot0.getHeading())
                .setTimeoutConstraint(500)
                .build();
        pickupCall1 = new FakeParameticCallback(0.2,()->{
            follower.setMaxPower(0.33);
            intakeOn = true;
            },follower);
        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup2[0],pickup2[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .setTimeoutConstraint(500)
                .build();
        pickupCall2 = new FakeParameticCallback(0.35,()->{
            follower.setMaxPower(0.36);
            intakeOn = true;
            },follower);

        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup3[0],pickup3[1]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup3[1].getHeading())
                .setTimeoutConstraint(500)
                .build();
        pickupCall3 = new FakeParameticCallback(0.33,()->{
            intakeOn = true;
        },follower);
        junoPath[0] = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],junoPose[0]))
                .setLinearHeadingInterpolation(pickup3[1].getHeading(),junoPose[0].getHeading())
                .build();
        junoPath[1] = follower.pathBuilder()
                .addPath(new BezierCurve(junoPose[0],junoPose[1],junoPose[2]))
                .setLinearHeadingInterpolation(junoPose[0].getHeading(),junoPose[2].getHeading(),0.8)
                .setTimeoutConstraint(500)
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1[1],shoot1))
                .setConstraints(shootConstraints)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .build();
        scoreCall1 = new FakeParameticCallback(0.9,()-> shootReady=true,follower);
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2[1],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .build();
        scoreCall2 = new FakeParameticCallback(0.9,()-> shootReady=true,follower);
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setLinearHeadingInterpolation(junoPose[2].getHeading(),shoot1.getHeading(),0.5)
                .build();
        scoreCall3 = new FakeParameticCallback(0.9,()-> shootReady=true,follower);
//        turretCall3 = new FakeParameticCallback(0.6,()-> trackingOn=true,follower);
        moveScore = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,movePoint))
                .setLinearHeadingInterpolation(shoot1.getHeading(), movePoint.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //region MAIN VARS
        int pathState = 0;
        int subState = 0;

        int shootingState = 0;
        boolean running = true;
        int shoot0change = 0;
        double flyOffset = 0;
        boolean flyAtSpeed = false;
        boolean motifRead = false;
        double cutoffTimer = 0;
        boolean canCutoffTimer = false;

        //BOOLEANS like stuff on or off
        boolean shutoffIntake = false;
        boolean transOn = false;
        boolean autoShootOn = false;
        boolean cutoffSpinPID = false;
        //endregion

        //region HARDWARE INFO
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class,"fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        trans = hardwareMap.get(DcMotor.class,"trans");

        //SERVOS
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        spin2 = hardwareMap.get(CRServo.class, "spin2");
        led = hardwareMap.get(Servo.class,"led");
        hood = hardwareMap.get(Servo.class,"hood");
        turret1 = hardwareMap.get(CRServo.class, "tu1");
        turret2 = hardwareMap.get(CRServo.class, "tu2");
        llservo = hardwareMap.get(Servo.class,"llservo");
//        tempServo = hardwareMap.get(Servo.class,"speedometer");

        //ENCODERS
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin1");
        turretEncoder = hardwareMap.get(AnalogInput.class, "tuen");

        //COLOR SENSOR
        color1 = hardwareMap.get(NormalizedColorSensor.class,"Color 1");
        color2 = hardwareMap.get(NormalizedColorSensor.class,"Color 2");

        //DIRECTIONS
        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.REVERSE);

        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        flywheel = new FlywheelPIDController(
                hardwareMap.get(DcMotorEx.class, "fly1"),
                hardwareMap.get(DcMotorEx.class, "fly2")
        );
        flywheel.teleopMultiplier = 0.88;
        //endregion

        //region CAMERA INIT
        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

//        initAprilTag();
//        setManualExposure(4, 200);
        //endregion

        //region INITIALIZE PEDRO
        createPoses();

        Pose2D ftcStartPose = PoseConverter.poseToPose2D(
                startPose,
                InvertedFTCCoordinates.INSTANCE
        );

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(ftcStartPose);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();

        StateVars.lastPose = startPose;
        limelightWallPos = pickup1[1].getX();
        //endregion

        flyOffset -= shoot0change;

        //region PRELIM READING MOTIF
        while(!isStarted()){ // this basically waits for start
            int april = readMotifLimelight();
            if(april!=-1) {
                if (april == 21) {
                    greenPos = 0;
                } else if (april == 22) {
                    greenPos = 1;
                } else if (april == 23) {
                    greenPos = 2;
                }
                StateVars.patternTagID = april;
                motifRead = true;

                String order = "";
                if(greenPos == 0) order = "GPP";
                else if(greenPos == 1) order = "PGP";
                else if(greenPos == 2) order = "PPG";
                telemetry.addData("Motif",greenPos + " ("+order+")");
            }
            if(!motifRead) telemetry.addData("Motif","Not Detected");
            telemetry.update();
        }
        //endregion

        //WAIT (put anything pre-start above motif reading)
        waitForStart();
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        while(opModeIsActive()){
            follower.update();
            StateVars.lastPose = follower.getPose();

            //region IMPORTANT VARS
            //needed at beginning of loop, don't change location
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            pidLastTimeMs = nowMs;

            if (dtSec <= 0.0) dtSec = 1.0 / 50.0;

            double turnInput = -gamepad1.right_stick_x;

            follower.update();
            Pose robotPose = follower.getPose();
            //endregion

            //region CHECK PCALLBACKS
            if(current[0]!=null){
                if(current[0].check()){
                    current[0] = null;
                }
            }
            if(current[1]!=null){
                if(current[1].check()){
                    current[1] = null;
                }
            }
            //endregion

            //region CUTOFF TIMER
            if(canCutoffTimer&&cutoffTimer<runtime.milliseconds()){
                canCutoffTimer = false;
                follower.breakFollowing();
            }
            //endregion

            if (runtime.milliseconds() > 28500) {
                pathState = 4;
            }
            //region PATH STUFF
            if(!follower.isBusy()&&runtime.milliseconds()>timeout){
                switch(pathState){
                    //region CYCLE ZERO (READ MOTIF)
                    case 0:
                        if(subState==0){
                            followPathPCallback(scorePath0,true,scoreCall0);
                            autoShootOn = true;
                            shootingState=0;
                            tuOffset = 8;
                            flyOffset = 35;

                            timeout = runtime.milliseconds()+2000;
                            subState++;
                        }
                        //AUTO SHOOTING is subState 1, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE ONE
                    case 1:
                        if(subState==0){
                            followPathPCallback(pickupPath1,false,pickupCall1);
                            tuOffset = 0;
                            flyOffset += shoot0change;

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            followPathPCallback(scorePath1,true,scoreCall1);
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 4, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE TWO
                    case 2:
                        if(subState==0){
                            followPathPCallback(pickupPath2,false,pickupCall2);
                            flyOffset = 20;

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            followPathPCallback(scorePath2,true,scoreCall2);
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE THREE
                    case 3:
                        if(subState==0){
                            followPathPCallback(pickupPath3,true,pickupCall3);
                            trackingOn = false;
                            canCutoffTimer = true;
                            cutoffTimer = runtime.milliseconds() + 5000;

                            subState++;
                        }
                        else if(subState==1){
                            timeout = runtime.milliseconds() + 300;
                            subState++;
                        }
                        else if(subState==2){
                            follower.followPath(junoPath[0],false);
                            canCutoffTimer = true;
                            cutoffTimer = runtime.milliseconds() + 1500;

                            subState++;
                        }
                        else if(subState==3){
                            follower.followPath(junoPath[1],true);
                            trackingOn = true;
                            canCutoffTimer = true;
                            cutoffTimer = runtime.milliseconds() + 3000;


                            subState++;
                        }
                        else if(subState==4){
                            timeout = runtime.milliseconds() + 150;
                            subState++;
                            canCutoffTimer = false;
                        }
                        //INTAKE is subState 0-4
                        else if(subState==5){
                            shutoffIntake = true;
                            trackingOn = true;
                            follower.setMaxPower(1);
                            followPathPCallback(scorePath3,true,scoreCall3);
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 4, resets subState, and increments pathState
                        break;
                    //endregion

                    case 4:
                        PathChain tempPath = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(),movePoint))
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), movePoint.getHeading())
                                .build();
                        follower.followPath(tempPath);
                        pathState++;
                        running=false;
                        break;
                }


            }
            //endregion

            //region LIMELIGHT SERVO
            llservo.setPosition(0.82);
            //endregion

            //region INTAKE
            char detectedColor = getRealColor();
            boolean present = isBallPresent();
            int currentSlot = indexToSlot(spindexerIndex);

            if(intakeOn){
                intake.setPower(1);
                transOn=false;
                if (present && savedBalls[currentSlot] == 'n' && spindexerAtTarget) {

                    if (detectedColor != 'n') {
                        savedBalls[currentSlot] = detectedColor;
                        spinClock();
                    }
                }

                if(spindexerFull()||(!follower.isBusy()&&pathState!=3)||shutoffIntake){
                    if(spindexerFull()){
                        intake.setPower(0);
                    }
                    if(pathState!=3){
                        follower.breakFollowing();
                        subState++;
                    }else if(!shutoffIntake){
                        follower.breakFollowing();
                        subState = 5;
                    }
                    intakeOn = false;
                    shutoffIntake = false;
                }
            }
            //endregion

            //region HOOD CONTROL
            //(angles must be negative for our direction)
            hood.setPosition((hoodAngle + hoodOffset)/355.0);
            //endregion

            //region SPINDEXER
            double targetAngle = SPINDEXER_POSITIONS[spindexerIndex];
            if(!cutoffSpinPID){
                updateSpindexerPID(targetAngle+ GlobalOffsets.spindexerOffset, dtSec);
            }
            //endregion

            //region SHOOT PREP
            if(autoShootOn&&shootingState==0){
                int greenIn=-1;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='g'){
                        greenIn=i;
                    }
                }
                if(greenIn==-1){
                    for(int i=0;i<3;i++){
                        if(savedBalls[i]=='n' || savedBalls[i]=='b'){
                            greenIn=i;
                        }
                    }
                }
                if(greenIn==-1) greenIn=0;

                int diff = (greenIn + greenPos) % 3;
                if(diff==0) spindexerIndex=4;
                else if(diff==1) spindexerIndex=0;
                else spindexerIndex=2;
                spindexerAtTarget=false;
//                timeout = runtime.milliseconds() + 300;

                shootingState++;
            }
            //endregion

            //region AUTO SHOOTING
            //prevent ball not firing
//            if(autoShootOn&&shootingState==1&&spindexerAtTarget) transOn = true;

            if(autoShootOn&&runtime.milliseconds()>timeout&&(shootReady||!follower.isBusy())){
                intake.setPower(0);
//                double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;
//                if(shootingState==1&&spindexerAtTarget&&avgSpeed > flySpeed * 0.94 && avgSpeed < flySpeed * 1.08){
                if(shootingState==1){
                    transOn = true;
                    if(flyAtSpeed){
                        spin1.setPower(0.85);
                        spin2.setPower(0.85);
                        cutoffSpinPID = true;

                        timeout=runtime.milliseconds()+900;
                        shootingState++;
                    }
                }
                else if(shootingState==2){
                    savedBalls[0]='n'; savedBalls[1]='n'; savedBalls[2]='n';

                    cutoffSpinPID = false;
                    shootReady = false;
                    autoShootOn = false;
                    shootingState++;
                    subState=0;
                    if (pathState != 3 || runtime.milliseconds() > 28000) {
                        pathState++;
                    }
                }
            }
            //endregion

            //region AUTO FLYSPEED/ANGLE
            //position and range
            double dx = goalX - follower.getPose().getX();
            double dy = goalY - follower.getPose().getY();
            double odomRange = Math.hypot(dx, dy);

            //velocity
            double velX = velocity.getXComponent();
            double velY = velocity.getYComponent();

            //finds the unit vector in the direction of the goal
            double unitVectorX = dx / odomRange;
            double unitVectorY = dy / odomRange;

            double totalSpeed = velocity.getMagnitude();

            //gives you the velocity in the direction of the goal (radial velocity)
            double radVel = (velX * unitVectorX) + (velY * unitVectorY);

            double adjustedRange = odomRange;
            if (Math.abs(radVel) > 5.0) { // threshold of 5 inches/second
                shotTime = interpolate(odomRange, ODOM_RANGE_SAMPLES, AIR_TIME) / 4.0;
                double radialDisplacement = radVel * shotTime;
                adjustedRange = odomRange - radialDisplacement;
            }

            //smooth range so values rnt erratic
            if (!isInitialized) {
                smoothedRange = adjustedRange;
                isInitialized = true;
            } else {
                smoothedRange = smooth(adjustedRange, smoothedRange);
            }

            // interpolate between measured values
            if (!flyHoodLock) {
                flySpeed = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, FLY_SPEEDS);
                hoodAngle = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES);
                hoodAngle = Math.max(hoodAngle, -140); //clamp to prevent it going too high
            }

            flySpeed += flyOffset;

            telemetry.addData("Odom Range", "%.1f inches", odomRange);
            telemetry.addData("Radial Velocity", "%.1f in/s", radVel);
            telemetry.addData("Adjusted Range", "%.1f inches", smoothedRange);
            //endregion

            //region FLYWHEEL
            //velocity

            if(flySpeed<1300) flySpeed = 1300;
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//            flySpeed = 0;
            flywheel.updateFlywheelPID(
                    flySpeed,
                    dtSec,
                    voltage
            );

            double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;

            // check if flywheel is at speed
            flyAtSpeed = Math.abs(flySpeed - flywheel.lastMeasuredVelocity) < 100;

//            if(avgSpeed >= flySpeed){
//                flyKd = 3;
//            }
//            tempServo.setPosition(avgSpeed/(flySpeed*2));
            //endregion

            //region GOAL TRACKING
            if (trackingOn) {
                tuPos = calcTuTarget(0, 0,
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        follower.getPose().getHeading()
                )
                        + (tuOffset * turretClock);
            }
            //endregion

            //region TURRET CONTROl
            //needs to stay right above the final calculations, otherwise will get overwritten
            if (!trackingOn) {
                //zeros position
                tuPos = normalizeDeg180(GlobalOffsets.turretZeroDeg);
            }

            double rawTurretTargetDeg = tuPos;
            //wraps position
            double safeTurretTargetDeg = applyTurretLimitWithWrap(rawTurretTargetDeg);
            tuPos = safeTurretTargetDeg;

            double targetVelDegPerSec = 0.0;

            //feedforward
            if (!lastTuTargetInit) {
                lastTuTarget = safeTurretTargetDeg;
                lastTuTargetInit = true;
            } else if (trackingOn) {
                double dTarget = normalizeDeg180(safeTurretTargetDeg - lastTuTarget);
                targetVelDegPerSec = dTarget / Math.max(dtSec, 1e-3);
                lastTuTarget = safeTurretTargetDeg;
            } else {
                // no FF when not tracking
                targetVelDegPerSec = 0.0;
                lastTuTarget = safeTurretTargetDeg;
            }

            updateTurretPIDWithTargetFF(tuPos, targetVelDegPerSec, dtSec);
            //endregion

            //region TRANSFER
            if(transOn){
                trans.setPower(1);
            }
            else{
                trans.setPower(0);
            }
            //endregion

            //region TELEMETRY
            if(!running) telemetry.addLine("Done!");

            telemetry.addData("Encoder Fly Speed",avgSpeed);
            telemetry.addData("path state", pathState);
            telemetry.addData("sub state",subState);
            telemetry.addData("shooting state",shootingState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Green Position",greenPos);
            telemetry.addData("actual fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("spindexer pos",spindexerIndex);
            telemetry.addData("spindexer at target",spindexerAtTarget);
            telemetry.update();
            //endregion
        }
    }

    //region HELPER METHODS
    //region TURRET AND LOCALIZATION

    private double calcTuTarget(double velX, double velY, double robotX, double robotY, double robotHeadingRad) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double futureRobotX = robotX + (velX * shotTime);
        double futureRobotY = robotY + (velY * shotTime);

        double futureDx = goalX - futureRobotX;
        double futureDy = goalY - futureRobotY;

        double headingToGoal = Math.toDegrees(Math.atan2(futureDy, futureDx));
        double robotHeading  = Math.toDegrees(robotHeadingRad);

        //actual turret angle needed
        double turretAngleReal = headingToGoal - robotHeading;

        //converts to angle servos need to turn to to achieve turret angle
        double servoAngle = GlobalOffsets.turretZeroDeg + (2 * turretAngleReal);

        return normalizeDeg180(servoAngle);
    }

    //TODO check ff and calculations for this, make as fast as possible
    private void updateTurretPIDWithTargetFF(double targetAngle, double targetVelDegPerSec, double dt) {
        double angle = getTurretAngleDeg();

        double error = -angleError(targetAngle, angle);

        //if (Math.abs(error) > 8.0) tuIntegral = 0;

        tuIntegral += error * dt;
        tuIntegral = clamp(tuIntegral, -tuIntegralLimit, tuIntegralLimit);

        double rawD = (error - tuLastError) / Math.max(dt, 1e-6);
        double d = 0.5 * tuLastD + 0.5 * rawD;
        tuLastD = d;

        double out = tuKp * error + tuKi * tuIntegral + tuKd * d;

        // stiction FF
        if (Math.abs(error) > tuToleranceDeg) out += tuKf * Math.signum(error);

        // target-rate FF (helps match d(turret)/d(target))
        out += tuKv * targetVelDegPerSec;

        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < tuDeadband) out = 0.0;

        turret1.setPower(out);
        turret2.setPower(out);

        tuLastError = error;

    }

    private double getTurretAngleDeg() {
        return normalizeDeg180(mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29));
    }

    //TODO make this wrap better
    private double applyTurretLimitWithWrap(double desiredDeg) {
        // Always reason in [-180, 180]
        desiredDeg = normalizeDeg180(desiredDeg);

        // Where the turret actually is right now (also [-180, 180])
        double currentDeg = getTurretAngleDeg();

        // Shortest signed rotation from current to desired (e.g. +20, -30, etc.)
        double errorToDesired = normalizeDeg180(desiredDeg - currentDeg);

        // "Ideal" next target if we perfectly matched desired in one step
        double candidateDeg = currentDeg + errorToDesired;

        // Hard safety clamp to keep off the wires
        return clamp(candidateDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
    }
    //endregion

    //region GENERAL MATH METHODS
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        return angle;
    }

    // Compute shortest signed difference between two angles
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    // Linear interpolation helper method
    private double interpolate(double x, double[] xValues, double[] yValues) {
        // Clamp to table bounds
        if (x <= xValues[0]) return yValues[0];
        if (x >= xValues[xValues.length - 1]) return yValues[yValues.length - 1];

        // Find surrounding points
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                // Linear interpolation formula
                double t = (x - xValues[i]) / (xValues[i + 1] - xValues[i]);
                return yValues[i] + t * (yValues[i + 1] - yValues[i]);
            }
        }
        return yValues[yValues.length - 1];
    }

    private double smooth(double newValue, double previousValue) {
        return ALPHA * newValue + (1 - ALPHA) * previousValue;
    }
    //endregion
    private void followPathPCallback(PathChain path, boolean holdEnd, FakeParameticCallback pCallback1){
        follower.followPath(path,holdEnd);
        current[0] = pCallback1;
    }
    private void followPathPCallback(PathChain path, boolean holdEnd, FakeParameticCallback pCallback1, FakeParameticCallback pCallback2){
        follower.followPath(path,holdEnd);
        current[0] = pCallback1;
        current[1] = pCallback2;
    }
    private double normalizeDeg180(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
    private char getRealColor(){
        char c1 = getDetectedColor1(color1);
        char c2 = getDetectedColor2(color2);

        if(c1=='p'||c2=='p'){
            return 'p';
        }
        if(c1=='g'||c2=='g'){
            return 'g';
        }
        return 'n';
    }

    private char getDetectedColor1(NormalizedColorSensor sensor){
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist1) {
            return 'n';
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha == 0) return 'n';
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }

    private char getDetectedColor2(NormalizedColorSensor sensor){
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        telemetry.addData("Distance X", dist);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist2) {
            return 'n';
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha == 0) return 'n';
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }

    private int indexToSlot(int index) {
        switch (index) {
            case 0: return 0;
            case 2: return 1;
            case 4: return 2;
            default: return -1;
        }
    }

    private int slotToIndex(int slot) {
        switch (slot) {
            case 0: return 0;
            case 1: return 2;
            case 2: return 4;
            default: return -1;
        }
    }
    private boolean isBallPresent() {
        double dist1 = ((DistanceSensor) color1).getDistance(DistanceUnit.CM);
        double dist2 = ((DistanceSensor) color2).getDistance(DistanceUnit.CM);

        NormalizedRGBA colors1 = color1.getNormalizedColors();
        NormalizedRGBA colors2 = color2.getNormalizedColors();

        boolean s1Detected = !Double.isNaN(dist1) && dist1 < GlobalOffsets.colorSensorDist1;
        boolean s2Detected = !Double.isNaN(dist2) && dist2 < GlobalOffsets.colorSensorDist2;

        if (colors1.alpha == 0) {
            s1Detected = false;
        }
        if (colors2.alpha == 0) {
            s2Detected = false;
        }
        return s1Detected || s2Detected;
    }
    //region SPINDEXER HELPERS
    public void spinClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex - 2 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
    }

    public void spinCounterClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex + 2) % SPINDEXER_POSITIONS.length;
    }

    private void updateSpindexerPID(double targetAngle, double dt) {
        if (!spindexerPidArmed) {
            lastError = 0;
            lastFilteredD = 0;
            integral = 0;
            spin1.setPower(0);
            spin2.setPower(0);
            spindexerPidArmed = true;
            return;
        }
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        double targetB = (targetAngle + 180.0) % 360.0;

        double errorA = -angleError(targetAngle, angle);
        double errorB = -angleError(targetB, angle);

        // compute shortest signed error [-180,180]
        double error = (Math.abs(errorA) <= Math.abs(errorB)) ? errorA : errorB;

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double rawD = (error - lastError) / Math.max(dt, 1e-6);

        double d = (lastFilteredD * 0.8) + (rawD * 0.2);
        lastFilteredD = d;

        // PIDF output (interpreted as servo power)
        double out = pidKp * error + pidKi * integral + pidKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += pidKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }

        spindexerAtTarget = (Math.abs(error) <= positionToleranceDeg+15);

        spin1.setPower(out);
        spin2.setPower(out);

        lastError = error;

        telemetry.addData("ENCODER VOLTAGE", spinEncoder.getVoltage());
        telemetry.addData("ANGLE", targetAngle);
        telemetry.addData("INTEGRAL", integral);
    }

    public void calculateNearestIndex() {
        double currentAngle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        int bestIndex = spindexerIndex;
        double minAbsError = 360.0;

        for (int i = 0; i < SPINDEXER_POSITIONS.length; i++) {
            double baseTarget = SPINDEXER_POSITIONS[i] + GlobalOffsets.spindexerOffset;

            // 1. Check the Normal Target
            double errorNormal = Math.abs(angleError(baseTarget, currentAngle));

            // 2. Check the "Ghost" Target (180 degrees away)
            // Since gearing is 1:2, this is the same physical position
            double ghostTarget = baseTarget + 180.0;
            double errorGhost = Math.abs(angleError(ghostTarget, currentAngle));

            // Find which one is closer
            double localMinError = Math.min(errorNormal, errorGhost);

            // Compare against the global best found so far
            if (localMinError < minAbsError) {
                minAbsError = localMinError;
                bestIndex = i;
            }
        }

        // Update the index.
        // The PID loop will automatically handle choosing between
        // the Normal or Ghost target again in the next frame.
        spindexerIndex = bestIndex;
    }
    //endregion

    private int readMotifLimelight(){
        LLResult result = limelight.getLatestResult();
        int numTags=0;
        int lastTagNum = 0;
        if (result != null && result.isValid()) {
            List <LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult detection : tags) {
                if (detection.getFiducialId() == 21||detection.getFiducialId() == 22||detection.getFiducialId() == 23) {
                    numTags++;
                    lastTagNum=detection.getFiducialId();
                }
            }
        }
        if(numTags==1) return lastTagNum;
        return -1;
    }
    private int readMotif(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int numTags=0;
        int lastTagNum = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21||detection.id == 22||detection.id == 23) {
                    numTags++;
                    lastTagNum=detection.id;
                }
            }
        }
        if(numTags==1) return lastTagNum;
        return -1;
    }

    private void pathToBall(double tx,double ty){
        double hypotenuse = Math.sqrt((tx*tx) + (ty*ty));
        telemetry.addData("hypotenuse",hypotenuse);
        double angle = Math.atan(tx/(ty-5));

        double distX = (Math.cos(follower.getHeading()-angle)*hypotenuse*Kball);
        double distY = (Math.sin(follower.getHeading()-angle)*hypotenuse*Kball);
        if(ty>50){
            distX = 0;
            distY = 0;
        }else if(hypotenuse>40){
            distX *= 1.7;
            distY *= 1.7;
        }else if(hypotenuse>30){
            distX *= 1.2;
            distY *= 1.2;
        }

        ballX = follower.getPose().getX() + distX;
        ballY = follower.getPose().getY() + distY;

        ballHeading = follower.getHeading()+(-angle*KballAngle);//in radians

        //prevent slamming into wall
        if(ballX<26){//144-118 = 26
            double distXchange = ballX-26;//negative
            double proportion = Math.abs(distXchange/distX);//positive
            double distYchange = distY * proportion;
            ballX -= distXchange;
            ballY -= distYchange;
        }

        Pose ballPose = new Pose(ballX,ballY,ballHeading);

        limelightPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),ballPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),ballPose.getHeading())
                .build();
    }
    private Pose limelightPose(){
        return new Pose();
    }

    private boolean spindexerFull(){
        for(int i=0;i<3;i++){
            if(savedBalls[i]=='n') return false;
        }
        return true;
    }
    private void initAprilTag() {

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                0, //x, right +, left -
                4, //y, forward +, back -
                12.5, //z up + down -
                0
        );

        YawPitchRollAngles orientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                0, //yaw, left and right
                -70, //pitch, forward, back
                180, //roll, orientation
                0
        );
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, orientation)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)

                .build();

        aprilTag.setDecimation(4);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void adjustDecimation(double range) {
        int newDecimation;

        if (range > 90) {
            newDecimation = 3;
        } else if (range > 50) {
            newDecimation = 3;
        } else {
            newDecimation = 4;
        }

        aprilTag.setDecimation(newDecimation);
        telemetry.addData("Decimation: ", "%d", newDecimation);

    }
    //endregion
}