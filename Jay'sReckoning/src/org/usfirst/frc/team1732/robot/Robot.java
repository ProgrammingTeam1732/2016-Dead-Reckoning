
package org.usfirst.frc.team1732.robot;

import java.util.ArrayList;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class Robot extends IterativeRobot {
	
	// Camera/Images
	private Image frame;
	private Image binaryFrame;
	private AxisCamera camera;
	
	// Motors
	private CANTalon left1; private CANTalon left2; private CANTalon left3;
	private CANTalon right1; private CANTalon right2; private CANTalon right3;
	
	// Constants
	private final double RATIO = 1.428571; // Goal width = 20 in. / goal height = 12 in. = 1.428
		
	// Color Limits
	private NIVision.Range PAR_HUE_RANGE = new NIVision.Range(120, 140); // Default hue range for goal
	private NIVision.Range PAR_SAT_RANGE = new NIVision.Range(50, 255); // Default saturation range for goal
	private NIVision.Range PAR_VAL_RANGE = new NIVision.Range(150, 255); // Default value range for goal
		
	// Search Limits
	private double RATIO_MIN = 1.0; // goal width = 20 in. / goal height = 12 in. = 1.428
	private double RATIO_MAX = 1.8; // Goal width = 20 in. / goal height = 12 in. = 1.428
	private double MIN_AREA = 500;
	private int PAR_LIMIT = 10;
		
	// Driving
	private enum STATES {
		FIRST_PARTICLE, POINT, TURN_ONE_CALC, TURN_ONE, MOVE_ONE_CALC, MOVE_ONE, TURN_TWO_CALC, TURN_TWO, MOVE_TWO_CALC, MOVE_TWO, DONE
	}
	private STATES state = STATES.FIRST_PARTICLE;
	private double turn_speed = 0.1;
	private double move_speed = 0.18;
	private int left_of_goal = 1;
	private Particle first_particle;
	private Particle last_particle;
	private Encoder encoder;
	private Wreckoning wreck = new Wreckoning();
	double angle;
    
    public void robotInit() {
		// Camera/Images
		camera = new AxisCamera("10.99.99.9");
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
		CameraServer.getInstance().setQuality(50);
		//camera.writeColorLevel(40);
		//camera.writeWhiteBalance(WhiteBalance.kFixedFluorescent1);
		//camera.writeExposureControl(ExposureControl.kAutomatic);
		//camera.writeBrightness(40);
		//camera.writeBrightness(5);
		//camera.writeExposureControl(ExposureControl.kAutomatic);
		// Camera
		SmartDashboard.putBoolean("Capture?", false);
		SmartDashboard.putBoolean("binaryFrame?", false);

		// Color limits
		SmartDashboard.putNumber("Particle hue min", PAR_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Particle sat min", PAR_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Particle val min", PAR_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Particle val max", PAR_VAL_RANGE.maxValue);
		
		// Search limits
		SmartDashboard.putNumber("Particle aspect min", RATIO_MIN);
		SmartDashboard.putNumber("Particle aspect max", RATIO_MAX);
		SmartDashboard.putNumber("Particle area min", MIN_AREA);
		SmartDashboard.putNumber("Particle Limit", PAR_LIMIT);
		
		// Driving
		SmartDashboard.putNumber("Direction", 0);
		SmartDashboard.putBoolean("do Aim?", false);
		SmartDashboard.putNumber("Turn Speed", 0);
		SmartDashboard.putNumber("Move Speed", 0);
		left1 = new CANTalon(11); left2 = new CANTalon(21); left3 = new CANTalon(22);
		right1 = new CANTalon(14); right2 = new CANTalon(12); right3 = new CANTalon(13);
		Encoder encoder = new Encoder(0,1);
		wreck.reset(encoder.getRaw(), encoder.getRaw());
		
		// Particle Display
		SmartDashboard.putNumber("Masked particles", 0);
		SmartDashboard.putNumber("Filtered particles", 0);
		SmartDashboard.putNumber("Area", 0);
		SmartDashboard.putNumber("Left", 0);
		SmartDashboard.putNumber("Right", 0);
		SmartDashboard.putNumber("Top", 0);
		SmartDashboard.putNumber("Bottom", 0);
		SmartDashboard.putNumber("Aspect", 0);
		SmartDashboard.putNumber("Distance",  0);
		SmartDashboard.putNumber("Direction", 0);
    }
    
    public void autonomousInit() {
 
    }

    public void autonomousPeriodic() {
    	
    }

    public void teleopPeriodic() {
    	if (SmartDashboard.getBoolean("Capture?", false)) {
			camera.getImage(frame);
			// Set color limits
			PAR_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Particle hue min", PAR_HUE_RANGE.minValue);
			PAR_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
			PAR_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Particle sat min", PAR_SAT_RANGE.minValue);
			PAR_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
			PAR_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Particle val min", PAR_VAL_RANGE.minValue);
			PAR_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle val max", PAR_VAL_RANGE.maxValue);

			// Threshold the image looking for yellow (Goal color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, PAR_HUE_RANGE, PAR_SAT_RANGE, PAR_VAL_RANGE);

			// Count and display particles
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numParticles);
			SmartDashboard.putNumber("Filtered particles", 0);

			// Set search limits
			RATIO_MIN = SmartDashboard.getNumber("Particle aspect min",   RATIO_MIN);
			RATIO_MAX = SmartDashboard.getNumber("Particle aspect max",   RATIO_MAX);
			MIN_AREA  = SmartDashboard.getNumber("Particle area min", MIN_AREA);
			PAR_LIMIT = (int) SmartDashboard.getNumber("Particle Limit", PAR_LIMIT);
			
			if (numParticles > 0) {					
				ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
				for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
					Particle par = new Particle(NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
					double temp = par.getAspect();
					if (par.getArea() > MIN_AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
						qualifyingParticles.add(par);
						//if (qualifyingParticles.size() > particleLimit) qualifyingParticles.remove(qualifyingParticles.size() - 1);
					}
				}
				SmartDashboard.putNumber("Filtered particles", qualifyingParticles.size());
				
				// Set driving limits
				turn_speed = SmartDashboard.getNumber("Turn Speed");
				move_speed = SmartDashboard.getNumber("Move Speed");
				
				// If it found a particle, point towards it, then drive towards it
				if(qualifyingParticles.size() > 0 && SmartDashboard.getBoolean("do Aim?")) {
					Particle bestPar = qualifyingParticles.get(0);
					for (int i = 1; i < qualifyingParticles.size(); i++)
						if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math.abs(RATIO - bestPar.getAspect()))
							bestPar = qualifyingParticles.get(i);
					SmartDashboard.putNumber("Area", bestPar.getArea());
					SmartDashboard.putNumber("Left", bestPar.getLeft()/640.0);
					SmartDashboard.putNumber("Right", bestPar.getRight()/640.0);
					SmartDashboard.putNumber("Top", bestPar.getTop()/480.0);
					SmartDashboard.putNumber("Bottom", bestPar.getBottom()/480.0);
					SmartDashboard.putNumber("Aspect", bestPar.getAspect());
					SmartDashboard.putNumber("Distance",  bestPar.getDistance());
					SmartDashboard.putNumber("Direction", bestPar.getDirection());
					drawRectangle(frame, bestPar);
					if(state == STATES.FIRST_PARTICLE) {
						first_particle = bestPar;
						state = STATES.POINT;
					} else if (state == STATES.POINT) {
						turn_speed = SmartDashboard.getNumber("Turn Speed");
						setMotors(turn_speed, -turn_speed);
						// If it has turned within 5% of the image
						if (Math.abs(bestPar.getDirection() - 0.5) < 0.05) {
							setMotors(0, 0);
							Timer.delay(0.5);
							last_particle = bestPar;
							state = STATES.TURN_ONE_CALC;
						}
					} else if (state == STATES.TURN_ONE_CALC) {
						angle = Math.acos((last_particle.getRight() - last_particle.getLeft())/20.0);
						wreck.reset(encoder.getRaw(), encoder.getRaw());
						double first_width = first_particle.getWidth();
						double last_width = last_particle.getWidth();
						left_of_goal = (last_width > first_width ? 1 : -1) * (last_particle.getDirection() < 0.5 ? 1 : -1);
						state = STATES.TURN_ONE;
					} else if (state == STATES.TURN_ONE) {
						turn_speed = SmartDashboard.getNumber("Turn Speed");
						setMotors(left_of_goal*turn_speed, left_of_goal*turn_speed);
						// If it has turned within 0.1 radian of target turn
						if (Math.abs((Math.PI/2 - angle) - wreck.getAngle(encoder.getRaw(), encoder.getRaw())) < 0.1 ) {
							setMotors(0, 0);
							Timer.delay(0.5);
							state = STATES.MOVE_ONE_CALC;
						}
					} else if (state == STATES.MOVE_ONE_CALC) {
						wreck.reset(encoder.getRaw(), encoder.getRaw());
					} else if (state == STATES.MOVE_ONE) {
						move_speed = SmartDashboard.getNumber("Move Speed");
						setMotors(move_speed, move_speed);
						// If within 5 inches of target distance
						if (Math.abs((Math.cos(Math.PI/2 - angle)*last_particle.getDistance())
							+ (Math.tan(angle)*last_particle.getWidth()/2.0)
							- wreck.getDistance(encoder.getRaw())) < 5) {
							setMotors(0,0);
							Timer.delay(0.5);
							state = STATES.TURN_TWO_CALC;
						}
					} else if (state == STATES.TURN_TWO_CALC){	
						angle = Math.PI/2;
						wreck.reset(encoder.getRaw(), encoder.getRaw());
						state = STATES.TURN_TWO;
					} else if (state == STATES.TURN_TWO) {
						turn_speed = SmartDashboard.getNumber("Turn Speed");
						setMotors(-1*left_of_goal*turn_speed, -1*left_of_goal*turn_speed);
						// If it has turned within 0.1 radian of target turn
						if (Math.abs(angle - wreck.getAngle(encoder.getRaw(), encoder.getRaw())) < 0.1 ) {
							setMotors(0, 0);
							Timer.delay(0.5);
							state = STATES.MOVE_TWO_CALC;
						}
					} else if (state == STATES.MOVE_TWO_CALC) {
						wreck.reset(encoder.getRaw(), encoder.getRaw());
					}
					else if (state == STATES.MOVE_TWO) {
						move_speed = SmartDashboard.getNumber("Move Speed");
						setMotors(move_speed, move_speed);
						// If within 5 inches of target distance
						if (Math.abs((Math.sin(Math.PI/2 - angle)*last_particle.getDistance()) + (Math.tan(angle)*last_particle.getWidth()/2.0) - wreck.getDistance(encoder.getRaw())) < 15) {
							setMotors(0,0);
							Timer.delay(0.5);
							state = STATES.DONE;
						}
					}
					else if (state == STATES.DONE) {
						
					}
				}
				else setMotors(0,0);
				
			}
			else setMotors(0, 0);
			// Send masked image to dashboard to assist in tweaking mask.
			if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
			else CameraServer.getInstance().setImage(frame);
		}
		else setMotors(0, 0);
    }
    
    private void setMotors(double l, double r) {
		left1.set(-l); left2.set(-l); left3.set(l); 
		right1.set(r); right2.set(r); right3.set(-r);
	}
    
    private void drawRectangle(Image image, Particle par) {
		NIVision.Rect rect = new NIVision.Rect((int) par.getTop(),
											   (int) par.getLeft(),
											   (int) (par.getBottom() - par.getTop()),
											   (int)(par.getRight() - par.getLeft()));
		NIVision.imaqDrawShapeOnImage(image,
									  image,
									  rect,
									  NIVision.DrawMode.PAINT_VALUE,
									  NIVision.ShapeMode.SHAPE_RECT, (float) 0x00FF00);
	}
    
}
