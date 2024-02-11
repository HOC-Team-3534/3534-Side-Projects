// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController xbox;
  TalonFX leftMotor, rightMotor;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    leftMotor = new TalonFX(0, "*");
    rightMotor = new TalonFX(20, "*");
    SmartDashboard.putNumber("Proportion", 1.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    rightMotor.set(xbox.getRightY() * SmartDashboard.getNumber("Proportion", 0.95));
    // leftMotor.setControl(new Follower(20, true));
    leftMotor.set(-xbox.getRightY());
  }
}
