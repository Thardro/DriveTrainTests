package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TankDriveTrain_GyroArcadeDrive extends Command {
	
	 //Inertial dampening
    final int LEFT = 0, RIGHT = 1;
    Timer timer = new Timer();
    double[] oldTime = new double[] {0, 0};
    double[] currentPower = new double[] {0,0};
    double maxAccel = 3.6; //motor power per second
	
	//Maximum amount robot will turn to drive forwards, any further will result in a reverse, degrees
	double maxForwardTurn = 100;
	
	private final double deadzone = 0.07;

    public TankDriveTrain_GyroArcadeDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
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
    	double x = Robot.oi.joystickRight.getX();
    	double y = Robot.oi.joystickRight.getY();
    	
    	x = (Math.abs(x) < deadzone)? 0 : x;
    	y = (Math.abs(y) < deadzone)? 0 : y;
    	
    	double speed = Math.max(Math.abs(x), Math.abs(y));
    	double angle = Math.atan2(x, -y) * 180 / Math.PI;
    	
    	//Calculating the amount the robot must turn to face the commanded direction
    	double angleError = angle - Robot.tankDriveTrain.getGyro();
    	while(angleError > 180) {
    		angleError -= 360;
    	}
    	while(angleError < -180) {
    		angleError += 360;
    	}
    	
    	if(!Robot.oi.right_2.get()) {
    		//Driving backwards if the angle error is greater than the maximum defined amount
    		if(angleError > maxForwardTurn) {
    			angleError -= 180;
    			speed *= -1;
    		}
    		if(angleError < -maxForwardTurn) {
    			angleError += 180;
    			speed *= -1;
    		}
    	}
    	double clockwiseCommand = angleError / 45;
    	//Normal arcade control when a button is held
    	if(Robot.oi.right_3.get()) {
    		speed = -y;
    		clockwiseCommand = x;
    	}
    	//Rezeroing of gyro when a button is pressed
    	if(Robot.oi.right_4.get()) {
    		Robot.tankDriveTrain.resetGyro();
    	}
    	
    	double leftPower = speed + clockwiseCommand;
    	double rightPower = speed - clockwiseCommand;
    	
    	leftPower = restrictAccel(leftPower, LEFT);
    	rightPower = restrictAccel(rightPower, RIGHT);
    	
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
