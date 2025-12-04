/*
 * COMPLETE, FIXED, AND MECANUM-COMPATIBLE AUTONOMOUS
 * FOR 4-MOTOR DRIVE (StarterBot 2025)
 *
 * This version is simplified to drive straight forward 26 inches off the line.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAutoLeave", group="StarterBot")
public class StarterBotAutoLeave extends OpMode {

    final double DRIVE_SPEED = 0.5;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    // Calculate TICKS_PER_MM once
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    private enum AutonomousState {
        INIT_DRIVE_FORWARD,
        WAIT_FOR_DRIVE,
        COMPLETE
    }
    private AutonomousState autonomousState;

    private final ElapsedTime driveTimer = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void init() {
        autonomousState = AutonomousState.INIT_DRIVE_FORWARD; // Start by initializing the movement

        // Hardware Mapping
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set directions: Left side reversed for driving forward
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Always reset encoders at init
        resetDriveEncoders();

        // Ensure motors hold position when power is zero
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized.");
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case INIT_DRIVE_FORWARD:
                driveForward(DRIVE_SPEED, 26, DistanceUnit.INCH);
                autonomousState = AutonomousState.WAIT_FOR_DRIVE;
                break;

            case WAIT_FOR_DRIVE:
                if (!leftFrontDrive.isBusy() && !rightFrontDrive.isBusy() &&
                        !leftBackDrive.isBusy() && !rightBackDrive.isBusy()) {
                    setAllPowers(0.0);
                    setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }

        telemetry.addData("Auto State", autonomousState);
        telemetry.addData("LF Current", leftFrontDrive.getCurrentPosition());
        telemetry.addData("LF Target", leftFrontDrive.getTargetPosition());
        telemetry.update();
    }

    void resetDriveEncoders() {
        // Must be in STOP_AND_RESET_ENCODER mode before setting target
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    void driveForward(double speed, double distance, DistanceUnit unit) {

        resetDriveEncoders(); // Reset position before setting a new target

        double mm = unit.toMm(distance);
        // Ticks is always positive for distance. Direction is handled by motor direction settings.
        int ticks = (int) (mm * TICKS_PER_MM);

        // All motors get the same target position for straight movement
        setAllTargetPositions(ticks, ticks, ticks, ticks);

        // Switch to RUN_TO_POSITION mode
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply speed
        setAllPowers(speed);
    }

    void setAllTargetPositions(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }

    void setAllModes(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    void setAllPowers(double pwr) {
        leftFrontDrive.setPower(pwr);
        rightFrontDrive.setPower(pwr);
        leftBackDrive.setPower(pwr);
        rightBackDrive.setPower(pwr);
    }
}