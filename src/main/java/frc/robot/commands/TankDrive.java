// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TankDrive() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    RobotContainer.driveTrain.setLeftMotorSpeed(0);
    RobotContainer.driveTrain.setRightMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      double speed = 0;
      double steering = RobotContainer.controller.getX(Hand.kRight);

      double rightSpeed = 0;
      double leftSpeed = 0;

      speed = RobotContainer.controller.getY(Hand.kLeft);

      speed *= -1;


      if (Math.abs(speed) < Constants.DriveConstants.DEADZONE)
        speed = 0;
  
      if (Math.abs(steering) < Constants.DriveConstants.DEADZONE)
        steering = 0;
  
        if (Math.abs(speed) > Constants.DriveConstants.MAX_SPEED)
        speed = Constants.DriveConstants.MAX_SPEED * (speed / Math.abs(speed));

        if (Math.abs(steering) > Constants.DriveConstants.MAX_SPEED)
        steering = Constants.DriveConstants.MAX_SPEED * (steering / Math.abs(steering));


      rightSpeed = speed - (steering * Constants.DriveConstants.STEERING_MULTIPLIER);
      leftSpeed = speed + (steering * Constants.DriveConstants.STEERING_MULTIPLIER);


      System.out.println(rightSpeed + ", " + leftSpeed);

      RobotContainer.driveTrain.setLeftMotorSpeed(leftSpeed);
      RobotContainer.driveTrain.setRightMotorSpeed(rightSpeed);

      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
