package edu.arizona.environment;

import java.util.List;
import java.util.Vector;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.SimulateAction;
import ros.pkg.oomdp_msgs.srv.SimulateAction.Response;
import edu.arizona.planning.mdp.OOMDPState;

// TODO: This still doesn't implement the whole specification
public class Environment {

	private static Environment instance_ = null;
	public static Environment getInstance() {
		if (instance_ == null) {
			instance_ = new Environment();
		}
		
		return instance_;
	}
	
	private NodeHandle nodeHandle_;
	private ros.ServiceClient<SimulateAction.Request, SimulateAction.Response, SimulateAction> simulateService_;
	
	// TODO: Should make private for singleton
	public Environment() {
		try {
			Ros.getInstance().init("environment_interface");
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
			System.out.println("...BUT IT'S REALLY OK");
		}
		nodeHandle_ = Ros.getInstance().createNodeHandle();
		
		// Let's try this for now
		simulateService_ = nodeHandle_.serviceClient("environment/simulate_action", new SimulateAction(), true);
	}

	// TODO: Seems like this is being called to often
	public OOMDPState simulateAction(OOMDPState state, String action) {
		SimulateAction.Request request = new SimulateAction.Request();
		request.action = action;
		
		request.state.object_states = new MDPObjectState[state.getObjectStates().size()];
		for (int i = 0; i < state.getObjectStates().size(); i++) {
			request.state.object_states[i] = state.getObjectStates().get(i).convertToROS();
		}
		// Not doing anything with relations for now, since the environment doesn't need them
		
		try {
			Response response = simulateService_.call(request);
			OOMDPState oomdpState = new OOMDPState(response.new_state);
			return oomdpState;
		} catch (RosException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	public List<String> getActions() {
		// TODO: Get these from the MDPDescription
		List<String> actions = new Vector<String>();
		actions.add("forward");
		actions.add("left");
		actions.add("right");
		return actions;
	}
}
