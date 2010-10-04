package edu.arizona.planning.mdp;

import edu.arizona.util.StateUtils;
import ros.pkg.wubble_mdp.msg.MDPObjectState;

public class LocationState extends edu.arizona.planning.mdp.MDPObjectState {
	
	public LocationState(MDPObjectState state) {
		super(state);
	}
	
	public LocationState(String name, double x, double y) {
		super(name, x, y);
	}

	@Override
	public MDPObjectState convertToROS() {
		MDPObjectState state = new MDPObjectState();
		state.mdp_class = MDPObjectState.LOCATION_CLASS;
		state.name = name;
		state.x = StateUtils.formatNumber(x);
		state.y = StateUtils.formatNumber(y);
		return state;
	}

	@Override
	public boolean equals(Object obj) {
		return this.toString().equals(obj.toString());
	}

	@Override
	public int hashCode() {
		return toString().hashCode();
	}

	@Override
	public String toString() {
		return "LOC:" + name + "@(" + StateUtils.formatNumber(x) + "," + StateUtils.formatNumber(y) + ")";
	}
}
