package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 *
 */
public class SwerveDriveTrain extends Subsystem {
	
	//Motor controllers
	CANTalon leftFrontRotate = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_FRONT_ROTATE);
	CANTalon rightFrontRotate = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_FRONT_ROTATE);
	CANTalon leftRearRotate = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_REAR_ROTATE);
	CANTalon rightRearRotate = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_REAR_ROTATE);
	
	CANTalon leftFrontDrive = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_FRONT_DRIVE);
	CANTalon rightFrontDrive = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_FRONT_DRIVE);
	CANTalon leftRearDrive = new CANTalon(RobotMap.DRIVETRAIN_TALON_LEFT_REAR_DRIVE);
	CANTalon rightRearDrive = new CANTalon(RobotMap.DRIVETRAIN_TALON_RIGHT_REAR_DRIVE);
    
	//Modules
	ModuleState[] modules = new ModuleState[4];
	
	//Sensors
	Gyro gyro = new ADXRS450_Gyro();
	
	//Constants
	public static final int ENC_COUNTS = 360;
	public static final double TRACKWIDTH = 2, WHEELBASE = 3;
	
	//Creating thread to make talon process motion profile buffer when points are available in upper buffer
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			leftFrontDrive.processMotionProfileBuffer();
			rightFrontDrive.processMotionProfileBuffer();
			leftRearDrive.processMotionProfileBuffer();
			rightRearDrive.processMotionProfileBuffer();
		}
	}
	
	Notifier notifier = new Notifier (new PeriodicRunnable());
	
	public SwerveDriveTrain() {
		modules[0] = new ModuleState(leftFrontRotate, leftFrontDrive, ModulePosition.FRONT_LEFT);
		modules[1] = new ModuleState(leftRearRotate, leftRearDrive, ModulePosition.REAR_LEFT);
		modules[2] = new ModuleState(rightFrontRotate, rightFrontDrive, ModulePosition.FRONT_RIGHT);
		modules[3] = new ModuleState(rightRearRotate, rightRearDrive, ModulePosition.REAR_RIGHT);
		
		//Beginning processing of motion profile
    	notifier.startPeriodic(0.005);
	}
	
	private class ModuleState {
		CANTalon rotateTalon, driveTalon;
		private double xPos, yPos;
		public double angle = 0, speed = 0, rawSpeed = 0;
		
		public ModuleState(CANTalon rotate, CANTalon drive, ModulePosition position) {
			rotateTalon = rotate;
			driveTalon = drive;
			
			initTalons();
			
			//Initializing coordinates of the module relative to the center of the robot
			switch(position) {
			case FRONT_LEFT:
				xPos = -TRACKWIDTH / 2;
				yPos = WHEELBASE / 2;
				break;
			case FRONT_RIGHT:
				xPos = TRACKWIDTH / 2;
				yPos = WHEELBASE / 2;
				break;
			case REAR_LEFT:
				xPos = -TRACKWIDTH / 2;
				yPos = -WHEELBASE / 2;
				break;
			case REAR_RIGHT:
				xPos = TRACKWIDTH / 2;
				yPos = -WHEELBASE / 2;
				break;
			}
		}
		
		private void initTalons() {
			driveTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
			driveTalon.configEncoderCodesPerRev(ENC_COUNTS);
			driveTalon.changeMotionControlFramePeriod(5);
			
			rotateTalon.setFeedbackDevice(FeedbackDevice.AnalogPot);
			rotateTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		}
		
		public CANTalon getDrive() {
			return driveTalon;
		}
		public CANTalon getRotate() {
			return rotateTalon;
		}
		
		public void setGoalPosition(double goalCenterX, double goalCenterY, double goalAngle) {
			double goalX = xPos * Math.cos(goalAngle) - yPos * Math.sin(goalAngle) + goalCenterX;
			double goalY = xPos * Math.sin(goalAngle) + yPos * Math.cos(goalAngle) + goalCenterY;
			
			rawSpeed = Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2));
			angle = Math.atan2(goalY - yPos, goalX - xPos);
		}
		
		public void normalizeSpeed(double[] speeds) {
			double max = 0;
			for(int i = 0; i < speeds.length; i++) {
				double currentSpeed = speeds[i];
				if(Math.abs(currentSpeed) > max) {
					max = Math.abs(currentSpeed);
				}
			}
			speed = rawSpeed / max;
		}
		
		public void goToGoalState() {
			setDriveTalon(speed);
		}
		
		private void setDriveTalon(double motorValue) {
			driveTalon.changeControlMode(TalonControlMode.PercentVbus);
			motorValue = safetyTest(motorValue);
			
			driveTalon.set(motorValue);
		}
		
		private double safetyTest(double motorValue) {
	        motorValue = (motorValue < -1) ? -1 : motorValue;
	        motorValue = (motorValue > 1) ? 1 : motorValue;
	        
	        return motorValue;
	    }
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void rotate(double angle) {
    	
    }
    
    public void setGoalPosition(double goalX, double goalY, double goalAngle) {
    	for(int i = 0; i < modules.length; i++) {
    		modules[i].setGoalPosition(goalX, goalY, goalAngle);
    	}
    	
    	double[] speeds = new double[modules.length];
    	for(int i = 0; i < speeds.length; i++) {
    		speeds[i] = modules[i].rawSpeed;
    	}
    	
    	for(int i = 0; i < modules.length; i++) {
    		modules[i].normalizeSpeed(speeds);
    	}
    }
    
    public enum ModulePosition {
    	FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT;
    }
}

