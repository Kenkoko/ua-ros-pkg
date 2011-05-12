package edu.arizona.verbs.environments.ww2d;

import java.text.NumberFormat;

import edu.arizona.simulator.ww2d.object.PhysicsObject;
import edu.arizona.simulator.ww2d.utils.enums.ObjectType;
import edu.arizona.verbs.shared.OOMDPObjectState;

public class State {
	
	private static final int gridConstant = 5;
	private static final int radialSectors = 8; 
	
	private static NumberFormat _format = NumberFormat.getInstance();
	static {
		_format.setMinimumFractionDigits(1);
		_format.setMaximumFractionDigits(1);
	}
	
	// TODO: These will need to be randomized
	public static float undiscretizeLoc(int loc) {
		return loc / (gridConstant * 1.0f);
	}
	
	public static float undiscretizeLoc(String locString) {
		return undiscretizeLoc(Integer.parseInt(locString));
	}
	
	public static float undiscretizeRot(int rot) {
		return rot * ((float) Math.PI / (radialSectors / 2));
	}
	
	public static float undiscretizeRot(String rotString) {
		return undiscretizeRot(Integer.parseInt(rotString));
	}
	
	public static int discretizeLoc(float x) {
		return Math.round(x * gridConstant);
	}
	
	public static int discretizeRot(float rot) {
		return Math.round(rot / (((float) Math.PI) / (radialSectors / 2))); 
	}
	
	// Range is -pi to pi
	public static float deltaAngle(double theta1, double theta2) {
		return (float) Math.atan2(Math.sin(theta1 - theta2), Math.cos(theta1 - theta2));
	}
	
	public static float discretizeV(float v) {
		float step = 0.3f;
		return step * Math.round(v / step);
	}
	
	String name;
	String className;
	float x, y;
	float heading;
	float vx, vy;
	float vtheta;
	
	public State(OOMDPObjectState objState) {
		name = objState.getName();
		className = objState.getClassName();
		
		x = undiscretizeLoc(objState.getValue("x"));
		y = undiscretizeLoc(objState.getValue("y"));
		heading = undiscretizeRot(objState.getValue("heading"));
					
		// TODO: Still need to discretize these, but probably with more precision?
		vx = Float.parseFloat(objState.getValue("vx"));
		vy = Float.parseFloat(objState.getValue("vy"));
		vtheta = Float.parseFloat(objState.getValue("vtheta"));
	}
	
	public State(PhysicsObject obj) {
		name = obj.getName();
		// TODO: Stupid mapping problem.  
		className = obj.getType().toString();
		if (obj.getType() == ObjectType.cognitiveAgent)
			className = "agent";
		
		x = obj.getPPosition().x;
		y = obj.getPPosition().y;
		heading = obj.getHeading();

		vx = obj.getBody().getLinearVelocity().x;
		vy = obj.getBody().getLinearVelocity().y;
		vtheta = obj.getBody().getAngularVelocity();
	}
	
	public OOMDPObjectState discretize() {
		OOMDPObjectState state = new OOMDPObjectState(name, className);
		
		state.setAttribute("x", String.valueOf(discretizeLoc(x)));
		state.setAttribute("y", String.valueOf(discretizeLoc(y)));
		state.setAttribute("heading", String.valueOf(discretizeRot(heading)));
		
		// TODO: Need to discretize further?
		state.setAttribute("vx", _format.format(discretizeV(vx)));
		state.setAttribute("vy", _format.format(discretizeV(vy)));
		state.setAttribute("vtheta", _format.format(discretizeV(vtheta)));
		
		return state;
	}
	
	public float computeDistanceTo(State other) {
		return (float) Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
	}
	
//	public float computeDistanceTo(State other) {
//		int deltaX = discretizeLoc(other.x) - discretizeLoc(this.x);
//		int deltaY = discretizeLoc(other.y) - discretizeLoc(this.y);
//		
////		return Math.max(deltaX, deltaY); // This is the chess distance
//		
//		if (deltaX == 0 && deltaY == 0) {
//			return 0.0f;
//		
//		} else {
//			float distance = (float) Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
//			
////			if (distance < 0.01) {
////				return 0.0f;
////			} else {
//				return distance;
////			}
//		}
//	}
	
	public float computeRelativeAngle(State other) {
		double deltaX = other.x - this.x;
		double deltaY = other.y - this.y;
		
		double directAngle = Math.atan2(deltaY, deltaX);

		return deltaAngle(directAngle, this.heading);
	}
}
