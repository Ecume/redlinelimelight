// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
  private final XboxController xbox;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private CANSparkMax m_leftFront = new CANSparkMax(8, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(30, MotorType.kBrushless);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftBack, m_leftFront);
  private CANSparkMax m_rightFront = new CANSparkMax(15, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(11, MotorType.kBrushless);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFront, m_rightBack);

  private DifferentialDrive m_robotDrive = new DifferentialDrive(leftGroup, rightGroup);
  private XboxController m_controller = new XboxController(0);
  
  private boolean obgectDetected = false;

  public Drivetrain(XboxController xbox) {
    this.xbox = xbox;
    rightGroup.setInverted(true);
    //LimelightHelpers.setPipelineIndex("limelight", 1);

    SendableChooser<Integer> pipeLine = new SendableChooser<>();
    pipeLine.setDefaultOption("aprilTag", Integer.valueOf(0));
    pipeLine.addOption("aprilTagZoom", Integer.valueOf(2));
    pipeLine.addOption("colorView", Integer.valueOf(1));
    SmartDashboard.putData(pipeLine);
    
    SmartDashboard.putBoolean("Something Detected", obgectDetected);
    pipeLine.onChange((pipeLineNum) -> {
      LimelightHelpers.setPipelineIndex("limelight", pipeLineNum);
    });

  }

    // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= .5;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = 1;
    double targetingForwardSpeed = 0.0;

    if(LimelightHelpers.getCurrentPipelineIndex("limelight") == 0){
      targetingForwardSpeed = LimelightHelpers.getCameraPose_TargetSpace("limelight")[2] * 0.4 *  kP;
    }
    else if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 1){
      targetingForwardSpeed = 2 / (LimelightHelpers.getTY("limelight") * 0.1 * kP);
    }
    //double targetingForwardSpeed = LimelightHelpers.getCameraPose_TargetSpace("limelight")[2] * 0.4 *  kP;
    //double targetingForwardSpeed = 2 / (LimelightHelpers.getTY("limelight") * 0.1 * kP);
    //if(Double.isInfinite(targetingForwardSpeed)) return 0;
    //System.out.println(targetingForwardSpeed);
    targetingForwardSpeed *= 0.345;
    targetingForwardSpeed *= 1.0;
    return targetingForwardSpeed;
  }

  @Override
  public void periodic() {
          System.out.println(
        limelight_range_proportional()
      );
    if(xbox.getLeftBumper()){
      System.out.println(
        limelight_range_proportional()
      );
      m_robotDrive.arcadeDrive(limelight_range_proportional(), limelight_aim_proportional()); 
    }
    else if (xbox.getRightBumper()) {
      
      m_robotDrive.arcadeDrive(-limelight_range_proportional(), limelight_aim_proportional());

    } else {
      m_robotDrive.arcadeDrive(0,0);
    }
    //System.out.println(LimelightHelpers.getTY("limelight"));
    //System.out.println("range " + limelight_range_proportional());
    //System.out.println("aim " + limelight_aim_proportional());
    // This method will be called once per scheduler run
        // Arcade drive with a given forward and turn rate
    
    int total=LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount;
    if(total>0){
    obgectDetected=true;
    }
    else{
      obgectDetected=false;
    }
    SmartDashboard.putBoolean("Something Detected", obgectDetected);
    

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
