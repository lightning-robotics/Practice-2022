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

  double rightSpeedLastFrame;
  double leftSpeedLastFrame;
  @Override
  public void execute() 
  {
      double speed = 0;
      double steering = RobotContainer.controller.getX(Hand.kRight);
      boolean aPressed = RobotContainer.controller.getAButton();
      boolean yPressed = RobotContainer.controller.getYButton();

      double rightSpeed = 0;
      double leftSpeed = 0;
      double maxSpeed = Constants.DriveConstants.MAX_SPEED;
      double currentSteeringMultiplier = Constants.DriveConstants.STEERING_MULTIPLIER;
      double currentSpeedMultiplier = Constants.DriveConstants.SPEED_MULTIPLIER;


      speed = RobotContainer.controller.getY(Hand.kLeft);


      speed *= -1;

      if(aPressed){
        currentSpeedMultiplier = Constants.DriveConstants.SLOW_SPEED_MULTIPLIER;
        currentSteeringMultiplier = Constants.DriveConstants.SLOW_STEERING_MULTIPLIER;
      }else if(yPressed){
        currentSpeedMultiplier = Constants.DriveConstants.FAST_SPEED_MULTIPLIER;
        currentSteeringMultiplier = Constants.DriveConstants.FAST_STEERING_MULTIPLIER;
      }


      if (Math.abs(speed) < Constants.DriveConstants.DEADZONE)
        speed = 0;
  
      if (Math.abs(steering) < Constants.DriveConstants.DEADZONE)
        steering = 0;
  
        if (Math.abs(speed) > maxSpeed)
        speed = maxSpeed * (speed / Math.abs(speed));

        if (Math.abs(steering) > maxSpeed)
        steering = maxSpeed * (steering / Math.abs(steering));


      
      speed *= currentSpeedMultiplier;
      

      rightSpeed = speed - (steering * currentSteeringMultiplier);
      leftSpeed = speed + (steering * currentSteeringMultiplier);
      
      if(aPressed && Math.abs(RobotContainer.controller.getY(Hand.kLeft)) < Constants.DriveConstants.DEADZONE && steering <= 0){
        rightSpeed = 0.05;
        leftSpeed = 0.05;
      }else{

        //WARNING this assumes that leftspeed and rightspeed are interwoven and not independent, so it just uses a single function
        if(Math.abs(leftSpeed - leftSpeedLastFrame) < 0.03){
          leftSpeedLastFrame = leftSpeed;
          rightSpeedLastFrame = rightSpeed;
        }

        leftSpeed = (leftSpeedLastFrame + leftSpeed)/2;
        rightSpeed = (rightSpeedLastFrame + rightSpeed)/2;
      }

      System.out.println(rightSpeed + ", " + leftSpeed);



      RobotContainer.driveTrain.setLeftMotorSpeed(leftSpeed);
      RobotContainer.driveTrain.setRightMotorSpeed(rightSpeed);

      leftSpeedLastFrame = leftSpeed;
      rightSpeedLastFrame = rightSpeed;

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
