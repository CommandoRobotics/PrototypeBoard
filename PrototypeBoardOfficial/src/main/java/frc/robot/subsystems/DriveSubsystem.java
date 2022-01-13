// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Declare the motor controllers
  Spark frontLeftSpark;
  Spark frontRightSpark;
  Spark rearLeftSpark;
  Spark rearRightSpark;

  // Declare the mecanum drive
  MecanumDrive drive;

  // Declare the NavX
  AHRS navX;

  // Define the drive rate limiters
  SlewRateLimiter yRateLimiter;
  SlewRateLimiter xRateLimiter;
  SlewRateLimiter rotateRateLimiter;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Instantiate the Sparks
    frontLeftSpark = new Spark(Constants.fLeftPort);
    frontRightSpark = new Spark(Constants.fRightPort);
    rearLeftSpark = new Spark(Constants.rLeftPort);
    rearRightSpark = new Spark(Constants.rRightPort);

    // Set which Sparks are inverted.
    frontLeftSpark.setInverted(Constants.fLeftInverted);
    frontRightSpark.setInverted(Constants.fRightInverted);
    rearLeftSpark.setInverted(Constants.rLeftInverted);
    rearRightSpark.setInverted(Constants.rRightInverted);

    // Instantiate the mecanum drive
    drive = new MecanumDrive(frontLeftSpark, rearLeftSpark, frontRightSpark, rearRightSpark);

    // Instantiate the NavX
    navX = new AHRS(SPI.Port.kMXP);
    navX.reset();

    // Instantiate the rate limiters
    yRateLimiter = new SlewRateLimiter(Constants.kMaxYAcceleration);
    xRateLimiter = new SlewRateLimiter(Constants.kMaxXAcceleration);
    rotateRateLimiter = new SlewRateLimiter(Constants.kMaxRotateAcceleration);
  }

  /**
   * Stop the drive motors
   */
  public void stop() {
    frontLeftSpark.set(0);
    frontRightSpark.set(0);
    rearLeftSpark.set(0);
    rearRightSpark.set(0);
  }

  /**
   * Set all motors to the same speed, which essentially drives forward
   * @param speed
   */
  public void driveForward(double speed) {
    frontLeftSpark.set(speed);
    frontRightSpark.set(speed);
    rearLeftSpark.set(speed);
    rearRightSpark.set(speed);
  }

  /**
   * Set the speed of the front left motor
   * @param speed
   */
  public void setFrontLeftSpeed(double speed) {
    frontLeftSpark.set(speed);
  }

  /**
   * Set the speed of the front right motor
   * @param speed
   */
  public void setFrontRightSpeed(double speed) {
    frontRightSpark.set(speed);
  }

  /**
   * Set the speed of the rear left motor
   * @param speed
   */
  public void setRearLeftSpeed(double speed) {
    rearLeftSpark.set(speed);
  }

  /**
   * Set the speed of the rear right motor
   * @param speed
   */
  public void setRearRightSpeed(double speed) {
    rearRightSpark.set(speed);
  }

  /**
   * Get the robot's current orientation
   * @return The robot's current orientation
   */
  public double getOrientation() {
    return navX.getAngle();
  }

  /**
   * Drive using Cartesian coordinates
   * @param ySpeed The forward/backward speed at which to drive
   * @param xSpeed The strafe speed at which to drive
   * @param rotation The rotation input
   */
  public void driveCartesian(double ySpeed, double xSpeed, double rotation) {
    drive.driveCartesian(yRateLimiter.calculate(ySpeed), xRateLimiter.calculate(xSpeed), rotateRateLimiter.calculate(rotation));
  }

    /**
   * Drive using Cartesian coordinates
   * @param ySpeed The forward/backward speed at which to drive
   * @param xSpeed The strafe speed at which to drive
   * @param rotation The rotation input
   */
  public void driveCartesianNoRateLimit(double ySpeed, double xSpeed, double rotation) {
    drive.driveCartesian(ySpeed, xSpeed, rotation);
  }

  /**
   * Drive field centrically using Cartesian coordinates
   * @param ySpeed The forward/backward speed at which to drive
   * @param xSpeed The strafe speed at which to drive
   * @param rotation The rotation input
   */
  public void driveCartesianFieldCentric(double ySpeed, double xSpeed, double rotation) {
    drive.driveCartesian(yRateLimiter.calculate(ySpeed), xRateLimiter.calculate(xSpeed), rotateRateLimiter.calculate(rotation), getOrientation());
  }
  
  /**
   * Drive field centrically using Cartesian coordinates
   * @param ySpeed The forward/backward speed at which to drive
   * @param xSpeed The strafe speed at which to drive
   * @param rotation The rotation input
   */
  public void driveCartesianFieldCentricNoRateLimit(double ySpeed, double xSpeed, double rotation) {
    drive.driveCartesian(ySpeed, xSpeed, rotation, getOrientation());
  }

  @Override
  public void periodic() {
  }
}
