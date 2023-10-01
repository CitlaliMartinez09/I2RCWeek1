// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; // A group of classes by frc that the code is requesting

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase { //CommandBase is the parent of TankDrive and its has things that the code wants so its is being called alongside the TankDrive
  public DriveTrain dt;
  public Joystick joy;

  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain dt, Joystick j) {
    this.dt = dt;//The "this" calls the dt that was mentioned before
    this.joy = j;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() { // This means the machine starts with no movement
    dt.tankDrive(0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // It starts to move to the left because there is more power in the right when it is executed
    double leftPowerRaw = joy.getRawAxis(1);

    double rightPowerRaw = joy.getRawAxis(5);

    dt.tankDrive(leftPowerRaw*-0.7, rightPowerRaw*-0.7);//multiplied by a negative number so it will go backwards
  }

  // Called once the command ends or is interrupted.
  @Override //When it ends, it is set to 0 so it doesn't move anymore
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override // It will be done when it returns false, if it returns true it is still running
  public boolean isFinished() {
    return false;
  }
}
