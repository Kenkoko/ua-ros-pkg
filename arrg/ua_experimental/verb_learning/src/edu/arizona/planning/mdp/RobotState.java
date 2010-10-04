package edu.arizona.planning.mdp;

import edu.arizona.util.StateUtils;
import ros.pkg.wubble_mdp.msg.MDPObjectState;


public class RobotState extends edu.arizona.planning.mdp.MDPObjectState {

	public String orientation;
	public double lastX;
	public double lastY;
	public String lastOrientation;
	
	public RobotState(MDPObjectState state) {
		super(state);
		this.orientation = state.orientation;
		this.lastX = Double.parseDouble(state.last_x);
		this.lastY = Double.parseDouble(state.last_y);
		this.lastOrientation = state.last_orientation;
	}
	
	public RobotState(String name, double x, double y, String orientation, double lastX, double lastY, String lastOrientation) {
		super(name, x, y);
		this.orientation = orientation;
		this.lastX = lastX;
		this.lastY = lastY;
		this.lastOrientation = lastOrientation;
	}

	@Override
	public MDPObjectState convertToROS() {
		MDPObjectState state = new MDPObjectState();
		state.name = name;
		state.mdp_class = MDPObjectState.ROBOT_CLASS;
		state.x = StateUtils.formatNumber(x);
		state.y = StateUtils.formatNumber(y);
		state.orientation = orientation;
		state.last_x = state.x;
		state.last_y = state.y;
		state.last_orientation = lastOrientation;
		return state;
	}

	@Override
	public String toString() {
		return "ROB:" + name + "@(" + StateUtils.formatNumber(x) + "," + StateUtils.formatNumber(y) + "," + orientation + "," + 
				StateUtils.formatNumber(lastX) + "," + StateUtils.formatNumber(lastY) + "," + lastOrientation + ")";
	}

}
