// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; //asking for a package, which is a group of classes. the package has things that the code needs

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase //DriveTrain is a subsystem
{
  private final WPI_TalonSRX leftDriveTalon; //final is a modifier and is used to keep WPI_TalonSRX the same no matter what
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);//Created a new object

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain");//.getTab is a getting method which means it obtains the information from Shuffleboard
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry();
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(true);
    rightDriveTalon.setInverted(false);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
public double ticksToMeters() {
  return (0.1524*Math.PI/4096)*getTicks();

}

  public double getAngle(){
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());//gets the percentage of voltage of the left motor
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());//gets the percentage of voltage of the right motor
    SmartDashboard.putNumber("Angle", navx.getAngle());

    SmartDashboard.putNumber("Right Talon Ticks", ticksToMeters());


    // We made this during Saturday session
    SmartDashboard.putNumber("Right Talon Ticks", rightDriveTalon.getSelectedSensorPosition());
// SmartDashboard is the class, .putNumber is the method and it is the action the class will do.
//key is something you can't delete, Right Talon Ticks is a String
//rightDriveTalon is also an object that we created, it has information on the talon's ticks
//.getSelectedSensorPosition is built in and is an example of a getting method, they obtain information of the object
//String is just the name of the object 

    SmartDashboard.putNumber("Left Talon Ticks", leftDriveTalon.getSelectedSensorPosition());

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}