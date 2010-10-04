package edu.arizona.planning.mdp;

public abstract class MDPObjectState implements Comparable<MDPObjectState> {
	
	public static MDPObjectState parseObjectMessage(ros.pkg.wubble_mdp.msg.MDPObjectState stateMessage) {
		if (ros.pkg.wubble_mdp.msg.MDPObjectState.ROBOT_CLASS.equals(stateMessage.mdp_class)) {
			return new RobotState(stateMessage);
		} else if (ros.pkg.wubble_mdp.msg.MDPObjectState.OBJECT_CLASS.equals(stateMessage.mdp_class)) {
			return new ObjectState(stateMessage);
		} else if (ros.pkg.wubble_mdp.msg.MDPObjectState.LOCATION_CLASS.equals(stateMessage.mdp_class)) {
			return new LocationState(stateMessage);
		} else {
			throw new RuntimeException("THIS DOES NOT HAVE A CLASS\n" + stateMessage);
		}
	}
	
	public String name;
	public double x;
	public double y;
	
	public MDPObjectState(ros.pkg.wubble_mdp.msg.MDPObjectState state) {
		name = state.name;
		x = Double.parseDouble(state.x);
		y = Double.parseDouble(state.y);
	}
	
	public MDPObjectState(String name, double x, double y) {
		this.name = name;
		this.x = x;
		this.y = y;
	}
	
	public abstract ros.pkg.wubble_mdp.msg.MDPObjectState convertToROS();

	@Override
	public int compareTo(MDPObjectState other) {
		return this.toString().compareTo(other.toString());
	}
	
	public abstract String toString();
}
