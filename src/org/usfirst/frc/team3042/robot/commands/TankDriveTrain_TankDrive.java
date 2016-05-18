package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TankDriveTrain_TankDrive extends Command {

	//Scale the joystick values to restrict maximum speed
    private final double speedScale = 1.0;
    
    private final double deadzone = 0.07;
    
    //Inertial dampening
    final int LEFT = 0, RIGHT = 1;
    Timer timer = new Timer();
    double[] oldTime = new double[] {0, 0};
    double[] currentPower = new double[] {0,0};
    double maxAccel = 3.6; //motor power per second
	
    public TankDriveTrain_TankDrive() {
    	requires(Robot.tankDriveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	Robot.tankDriveTrain.setMotors(0, 0);
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double leftPower = -Robot.oi.joystickLeft.getY() * speedScale;
        double rightPower = -Robot.oi.joystickRight.getY() * speedScale;
        
        leftPower = (Math.abs(leftPower) < deadzone)? 0 : leftPower;
        rightPower = (Math.abs(rightPower) < deadzone)? 0 : rightPower;
        
        leftPower = restrictAccel(leftPower, LEFT);
        rightPower = restrictAccel(rightPower, RIGHT);
        
        if (Robot.oi.left_1.get()){
            rightPower = leftPower;
        }
        else if (Robot.oi.right_1.get()){
            leftPower = rightPower;
        }
        
        
        Robot.tankDriveTrain.setMotors(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.logger.log("End", 1);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.logger.log("Interrupt", 1);
    }
    
    private double restrictAccel (double goalValue, int SIDE) {
        double currentTime = timer.get();
        double dt = currentTime - oldTime[SIDE];
        oldTime[SIDE] = currentTime;
        
        double maxDSpeed = maxAccel * dt;
        maxDSpeed *= (goalValue >= currentPower[SIDE])? 1 : -1;
         
        currentPower[SIDE] = (Math.abs(maxDSpeed) > Math.abs(goalValue - currentPower[SIDE]))? 
                goalValue : maxDSpeed + currentPower[SIDE];
        return currentPower[SIDE];
    }
}
