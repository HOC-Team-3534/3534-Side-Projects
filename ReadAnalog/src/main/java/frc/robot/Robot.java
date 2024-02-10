// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
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
  AnalogInput pot = new AnalogInput(0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Lowest Voltage", 0.003662109);
    SmartDashboard.putNumber("Highest Voltage", 4.869384267);
    SmartDashboard.putNumber("Range in Degrees", 3600);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Potentiometer Voltage Reading", pot.getVoltage());
    var lowVoltage = SmartDashboard.getNumber("Lowest Voltage", 0.003662109);
    var highVoltage = SmartDashboard.getNumber("Highest Voltage", 4.869384267);
    var voltageRange = highVoltage - lowVoltage;
    var degreesRange = SmartDashboard.getNumber("Range in Degrees", 3600);
    var degreesPerVolt = degreesRange / voltageRange;
    SmartDashboard.putNumber("Potentiometer Calculated Position in Degrees",
        degreesPerVolt * (pot.getVoltage() - lowVoltage));
  }
}
