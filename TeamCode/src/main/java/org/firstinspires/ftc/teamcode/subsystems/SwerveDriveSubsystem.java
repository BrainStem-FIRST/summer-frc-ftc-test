package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final MecanumDrive mechanumDrive;

    private final SwerveDriveKinematics brainSTEMSwerve;

    private final Encoder frontLeft, frontRight, backLeft, backRight;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public SwerveDriveSubsystem(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, final double diameter) {
        frontLeft = frontLeftMotor.encoder;
        frontRight = frontRightMotor.encoder;
        backLeft = backLeftMotor.encoder;
        backRight = backRightMotor.encoder;

        brainSTEMSwerve = new SwerveDriveKinematics();


        WHEEL_DIAMETER = diameter;

        mechanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public SwerveDriveSubsystem(HardwareMap hMap, final String frontLeftMotorName, String frontRightMotorName,
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
        mechanumDrive.driveRobotCentric(strafe, forward, turnSpeed);
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



    public void resetAllEncoders() {
        frontRight.reset();
        frontLeft.reset();
        backRight.reset();
        backLeft.reset();
    }

    public double getAverageEncoderDistance() {
        return (getFrontLeftEncoderDistance() + getFrontRightEncoderDistance() + getbackLeftEncoderDistance() + getbackRightEncoderDistance()) / 4.0;
    }

}
