package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="West Coast")
public class WestCoast extends OpMode {
    private double LeftMotorPower;
    private double RightMotorPower;

    private double IntakePower;

    private double LinearSlidePower;

    private DcMotor LeftMotor;
    private DcMotor RightMotor;

    private DcMotor LeftIntakeMotor;
    private DcMotor RightIntakeMotor;

    private DcMotor Gripper;
    private DcMotor LinearSlideMotor;
    private DcMotor Arm;

    private final double MaxPower = 1;

    private final double MaxDrivePower = Math.min(0.7, MaxPower);
    private final double MaxIntakePower = Math.min(0.7, MaxPower);

    private final double LinearSlideMotorMaxPower = Math.min(1, MaxPower);

    private ElapsedTime runtime;

    private final String LeftMotorName = "left_drive";
    private final String RightMotorName = "right_drive";
    private final String LeftIntakeName = "left_intake";
    private final String RightIntakeName = "right_intake";
    private final String LinearActuatorName = "linear_actuator";
    private final String GripperName = "gripper";
    private final String ArmName = "arm";

    @Override
    public void init() {
        // Drive motor configuration
        LeftMotor = hardwareMap.get(DcMotor.class, LeftMotorName);
        RightMotor = hardwareMap.get(DcMotor.class, RightMotorName);

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftMotorPower = 0;
        RightMotorPower = 0;

        // Intake motor configuration
        LeftIntakeMotor = hardwareMap.get(DcMotor.class, LeftIntakeName);
        RightIntakeMotor = hardwareMap.get(DcMotor.class, RightIntakeName);

        LeftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakePower = 0;

        // Slide motor configuration
        LinearSlideMotor = hardwareMap.get(DcMotor.class, LinearActuatorName);

        LinearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        LinearSlidePower = 0;

        // Block Gripper servo configuration
        Gripper = hardwareMap.get(DcMotor.class, GripperName);


        Arm = hardwareMap.get(DcMotor.class, ArmName);

        // Timer setup
        runtime = new ElapsedTime();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Motor control calculations
        LeftMotorPower = Math.min(MaxDrivePower, Math.max(gamepad1.left_stick_y, -MaxDrivePower));
        RightMotorPower = Math.min(MaxDrivePower, Math.max(gamepad1.right_stick_y, -MaxDrivePower));

        LeftMotorPower /= gamepad1.left_trigger+1;
        RightMotorPower /= gamepad1.right_trigger+1;

        // Gripper reading
        IntakePower = Math.min(MaxIntakePower, gamepad2.right_trigger);
        IntakePower -= Math.min(MaxIntakePower, gamepad2.left_trigger);

        // Motor power
        LeftMotor.setPower(LeftMotorPower);
        RightMotor.setPower(RightMotorPower);

        LeftIntakeMotor.setPower(IntakePower);
        RightIntakeMotor.setPower(IntakePower);

        // Linear slide power
        if (gamepad2.dpad_up) {
            LinearSlidePower = LinearSlideMotorMaxPower;
        } else if (gamepad2.dpad_down) {
            LinearSlidePower = -LinearSlideMotorMaxPower;
        } else {
            LinearSlidePower = 0;
        } LinearSlideMotor.setPower(LinearSlidePower);

        // Gripper swivel servo
        if (gamepad2.a) {
            Arm.setPower(1);
        } else if (gamepad2.x) {
            Arm.setPower(-1);
        } else {
            Arm.setPower(0);
        }

        if (gamepad2.right_bumper) {
            Gripper.setPower(1);
        } else if (gamepad2.left_bumper) {
            Gripper.setPower(-1);
        } else {
            Arm.setPower(0);
        }

        // Telemetry
        telemetry.addData("Runtime: ", runtime);
        telemetry.addLine();
        telemetry.addData("Left Power: ", LeftMotorPower);
        telemetry.addData("Right Power: ", RightMotorPower);
        telemetry.addLine();
        telemetry.addData("Intake Power: ", IntakePower);
        telemetry.addLine();
        telemetry.addData("Linear Slide Power: ", LinearSlidePower);
        telemetry.update();
    }

    @Override
    public void stop() {
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
}