// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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
  private final XboxController xbox = new XboxController(0);
  private final TalonFX turret = new TalonFX(15);
  private final AnalogInput pot = new AnalogInput(0);

  private final MotionMagicVoltage turret_mmRequest = new MotionMagicVoltage(0);

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final NetworkTable potStats = inst.getTable("Potentiometer");
  private final DoublePublisher potVoltage = potStats.getDoubleTopic("Potentiometer Voltage").publish();
  private final DoubleSubscriber potDegreesPerVolt = potStats.getDoubleTopic("Potentiometer Degrees Per Volt")
      .subscribe(0);

  private final NetworkTable turretStats = inst.getTable("Turret");
  private final DoubleSubscriber turretVoltageCenterOffset = turretStats
      .getDoubleTopic("Center of Turret Rotation Voltage Offset").subscribe(2.5);
  private final DoublePublisher turretPositionDegrees = turretStats.getDoubleTopic("Turret Position Degrees").publish();
  private final DoublePublisher turretVelocityDegrees = turretStats.getDoubleTopic("Turret Velocity Deg/S").publish();

  private int printCount;
  private boolean turretPositionCalibrated;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    potDegreesPerVolt.getTopic().publish().set(742.26);
    turretVoltageCenterOffset.getTopic().publish().set(2.5);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 0.5;
    mm.MotionMagicAcceleration = 5;
    mm.MotionMagicJerk = 30;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 100;
    slot0.kI = 0;
    slot0.kD = 1;
    slot0.kV = 8.7184; // TODO Tune
    slot0.kS = 0.6373; // TODO Tune

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 125; // 125:1 rotor to sensor is 1:1, sensor to output shaft (mechanism) is 125:1,
                                      // which is represented here at a ration of sensor over mechanism

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = turret.getConfigurator().apply(cfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
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
    if (printCount++ > 10) {
      printCount = 0;
      var potVoltage = pot.getVoltage();
      this.potVoltage.set(potVoltage);

      var turretDegrees = Rotation2d.fromRotations(turret.getPosition().getValueAsDouble()).getDegrees();
      var turretDegreesPerSecond = Rotation2d.fromRotations(turret.getVelocity().getValueAsDouble()).getDegrees();

      turretPositionDegrees.set(turretDegrees);
      turretVelocityDegrees.set(turretDegreesPerSecond);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var rawInput = xbox.getRightX();
    var deadband = 0.2;
    var deadbandedInput = (rawInput / Math.abs(rawInput))
        * (Math.max(Math.abs(rawInput) - deadband, 0) / (1 - deadband));

    var targetDegrees = deadbandedInput * 360;

    if (targetDegrees > 350)
      targetDegrees = 350;
    if (targetDegrees < -350)
      targetDegrees = -350;

    var targetRotations = Rotation2d.fromDegrees(targetDegrees).getRotations();

    if (turretPositionCalibrated) {
      turret
          .setControl(turret_mmRequest.withPosition(targetRotations).withSlot(0));
    }

    if (xbox.getStartButton()) {
      var degreesPerVolt = potDegreesPerVolt.get();

      var currentTurretDegrees = degreesPerVolt
          * (pot.getVoltage() - turretVoltageCenterOffset.get());

      turret.setPosition(Rotation2d.fromDegrees(currentTurretDegrees).getRotations());

      turretPositionCalibrated = true;
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }
}
