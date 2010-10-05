package edu.arizona.environment;

import java.util.List;
import java.util.Vector;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.wubble_mdp.msg.MDPObjectState;
import ros.pkg.wubble_mdp.srv.SimulateAction;
import ros.pkg.wubble_mdp.srv.SimulateAction.Response;
import edu.arizona.planning.mdp.MDPState;

public class Environment {

	private NodeHandle nodeHandle_;
	private ros.ServiceClient<SimulateAction.Request, SimulateAction.Response, SimulateAction> simulateService_;
	
	public Environment() {
		Ros.getInstance().init("environment_interface");

		nodeHandle_ = Ros.getInstance().createNodeHandle();
		
		simulateService_ = nodeHandle_.serviceClient("environment/simulate_action", new SimulateAction());
	}
	
	public MDPState simulateAction(MDPState state, String action) {
		SimulateAction.Request request = new SimulateAction.Request();
		request.action = action;
		
		request.state.object_states = new MDPObjectState[state.getObjectStates().size()];
		for (int i = 0; i < state.getObjectStates().size(); i++) {
			request.state.object_states[i] = state.getObjectStates().get(i).convertToROS();
		}
		
		// Not doing anything with relations for now, since the environment doesn't need them
		
		try {
			Response response = simulateService_.call(request);
			return new MDPState(response.state);
		} catch (RosException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	public List<String> getActions() {
		List<String> actions = new Vector<String>();
		actions.add("forward");
		actions.add("left");
		actions.add("right");
//		actions.add("wait");
		return actions;
	}
	
	/**
	 * @param args
	 * @throws RosException 
	 */
	public static void main(String[] args) throws RosException {
		
		
		
//		MDPObjectState objState = new MDPObjectState();
//		objState.mdp_class = MDPObjectState.ROBOT_CLASS;
//		objState.name = "robot";
//		objState.x = "2.0";
//		objState.y = "0.0";
//		objState.orientation = "E";
		
		
//		simulate.shutdown();
//		System.out.println(response.state);
	}

}
