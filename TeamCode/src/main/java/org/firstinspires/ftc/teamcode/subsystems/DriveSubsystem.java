package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive brainSTEM_DT;

    private final Encoder frontLeft, frontRight, backLeft, backRight;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(MotorEx p_frontLeftMotor, MotorEx p_frontRightMotor, MotorEx p_backLeftMotor, MotorEx p_backRightMotor, final double diameter) {
        frontLeft = p_frontLeftMotor.encoder;
        frontRight = p_frontRightMotor.encoder;
        backLeft = p_backLeftMotor.encoder;
        backRight = p_backRightMotor.encoder;


        WHEEL_DIAMETER = diameter;

        brainSTEM_DT = new MecanumDrive(p_frontLeftMotor, p_frontRightMotor, p_backLeftMotor, p_backRightMotor);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public DriveSubsystem(HardwareMap hMap, final String frontLeftMotorName, String frontRightMotorName,
                          String backLeftMotorName, String backRightMotorName,
                          final double diameter) {
        this(new MotorEx(hMap, frontLeftMotorName), new MotorEx(hMap, frontRightMotorName),
                new MotorEx(hMap, backLeftMotorName), new MotorEx(hMap, backRightMotorName), diameter);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param forward the commanded forward movement
     * @param turnSpeed the commanded rotation
     */
    public void drive(double strafe, double forward, double turnSpeed) {
        brainSTEM_DT.driveRobotCentric(strafe, forward, turnSpeed);
    }

    public double getFrontLeftEncoderVal() {
        return frontLeft.getPosition();
    }

    public double getFrontRightEncoderVal() {
        return frontRight.getPosition();
    }

    public double getbackRightEncoderDistance() {
        return backRight.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getbackLeftEncoderDistance() {
        return backLeft.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getBackLeftEncoderVal() {
        return backLeft.getPosition();
    }

    public double getBackRightEncoderVal() {
        return backRight.getPosition();
    }

    public double getFrontRightEncoderDistance() {
        return frontRight.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getFrontLeftEncoderDistance() {
        return frontLeft.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }



    public void resetEncoders() {
        frontRight.reset();
        frontLeft.reset();
        backRight.reset();
        backLeft.reset();
    }

    public double getAverageEncoderDistance() {
        return (getFrontLeftEncoderDistance() + getFrontRightEncoderDistance() + getbackLeftEncoderDistance() + getbackRightEncoderDistance()) / 4.0;
    }

}
