package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RKR Autonomous (Left Side)")
public class Autonomous_R extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final String VUFORIA_KEY = "ARjoxi3/////AAABmXAUt9n1wEjDvzITTR3QybQ8V/6WWaLzzVteSXjxrJxxyilUnEB7VpV53J57ifvzM7/fbFhFTg7GNDdZs+QVVoZQ7Pt0SjP9THhkkR9zj42ztarG+IUc8Id5pW1juuKzHyiKz4sJWmDI6I3RVsa9R/H4rWUcdQUvWBF7X/lTOSMOHHPuz09pIDtLU/BTpL47hjCefksIY4VJ0KurxkrwcvbHufGh50i7j5jW74dt/NJrG2NPkt134vmeyzLI3/cBNIsOOZrWetxzQ0cXPAkS+3MItc53kFV+RTYt+C2t2xlaydfbAvboHFXExl+nHzh5AA+12JbvK3ikBBOnmORkkoDD0ncscgD6lKeEkQvOl0zw";
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private DcMotor LeftMotor;
    private DcMotor RightMotor;

    private DcMotor LeftIntakeMotor;
    private DcMotor RightIntakeMotor;

    private Player player;

    static final double     COUNTS_PER_MOTOR_REV   = 1440 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private void vuforiaInit() {
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables.addAll(targetsSkyStone);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuparameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuparameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(vuparameters);
    }

    public Autonomous_R() {
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double getHeading() {
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void doTelemetry() {
        telemetry.addData("Heading: ", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Roll: ", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("Pitch: ", formatAngle(angles.angleUnit, angles.thirdAngle));

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target: ", trackable.getName());
                targetVisible = true;
            }
        } if (!targetVisible) {
            telemetry.addData("Visible Target: ", "None");
        }

        telemetry.update();
    }

    public void autoTurn(double degrees, double power) {
        double startHeading = getHeading();
        double endHeading = startHeading+degrees;
        if (endHeading < -180) {endHeading += 360;}
        if (endHeading > 180) {endHeading -= 360;}

        if (degrees < 0) {
            LeftMotor.setPower(power);
            RightMotor.setPower(-power);
            while (getHeading() > endHeading && opModeIsActive()) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                doTelemetry();
            }
            LeftMotor.setPower(0);
            RightMotor.setPower(0);
        } else {
            LeftMotor.setPower(-power);
            RightMotor.setPower(power);
            while (getHeading() < endHeading && opModeIsActive()) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                doTelemetry();
            }
            LeftMotor.setPower(0);
            RightMotor.setPower(0);
        }
    }

    public void autoDrive(double inches, double power, double times) {
        ElapsedTime timer = new ElapsedTime();

        int newLeftTarget = LeftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRightTarget = RightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        LeftMotor.setTargetPosition(newLeftTarget);
        RightMotor.setTargetPosition(newRightTarget);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftMotor.setPower(Math.abs(power));
        RightMotor.setPower(Math.abs(power));

        timer.reset();
        while (opModeIsActive() && (LeftMotor.isBusy() && RightMotor.isBusy()) && timer.seconds()<times) {
            telemetry.addData("Targets: ",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Current Pos: ",  "Running at %7d :%7d",
                    LeftMotor.getCurrentPosition(),
                    RightMotor.getCurrentPosition());
        }

        LeftMotor.setPower(0);
        RightMotor.setPower(0);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initialize() {
        // Initialization code

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        player = new Player(hardwareMap);
        player.setSong("ucanttouchthis");

        // As it says
        vuforiaInit();
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        player.start();

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();

            doTelemetry();
        }
        while (opModeIsActive()) {}
        player.stop();
    }
}