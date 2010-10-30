package edu.arizona.verbs.environment;

import java.util.List;
import java.util.Vector;

import edu.arizona.verbs.mdp.OOMDPObjectState;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.StateConverter;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;
import ros.pkg.oomdp_msgs.srv.PerformAction;
import ros.pkg.oomdp_msgs.srv.SimulateAction;
import ros.pkg.oomdp_msgs.srv.SimulateAction.Response;

public class GazeboEnvironment implements Environment {
	private NodeHandle nodeHandle_;
	private ros.ServiceClient<SimulateAction.Request, SimulateAction.Response, SimulateAction> simulateService_;
	private ros.ServiceClient<InitializeEnvironment.Request, InitializeEnvironment.Response, InitializeEnvironment> initializeService_;
	private ros.ServiceClient<PerformAction.Request, PerformAction.Response, PerformAction> performService_;
	
	public GazeboEnvironment() {
		try {
			Ros.getInstance().init("environment_interface");
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
			System.out.println("...BUT IT'S REALLY OK");
		}
		nodeHandle_ = Ros.getInstance().createNodeHandle();
		
		simulateService_ = nodeHandle_.serviceClient("environment/simulate_action", new SimulateAction(), true);
		initializeService_ = nodeHandle_.serviceClient("environment/initialize", new InitializeEnvironment(), true);
		performService_ = nodeHandle_.serviceClient("environment/perform_action", new PerformAction(), true);
	}
	
	@Override
	public OOMDPState simulateAction(OOMDPState state, String action) {
		SimulateAction.Request request = new SimulateAction.Request();
		request.action = action;
		
		request.state.object_states = new MDPObjectState[state.getObjectStates().size()];
		for (int i = 0; i < state.getObjectStates().size(); i++) {
			request.state.object_states[i] = StateConverter.objStateToMsg(state.getObjectStates().get(i));
		}
		
		try {
			Response response = simulateService_.call(request);
			OOMDPState oomdpState = StateConverter.msgToState(response.new_state);
			return oomdpState;
		} catch (RosException e) {
			e.printStackTrace();
			return null;
		}
	}
	
	@Override
	public List<String> getActions() {
		// TODO: Get these from the MDPDescription
		List<String> actions = new Vector<String>();
		actions.add("forward");
		actions.add("left");
		actions.add("right");
		return actions;
	}

	@Override
	public OOMDPState performAction(String action) {
		PerformAction.Request req = new PerformAction.Request();
		req.action = action;
		try {
			ros.pkg.oomdp_msgs.srv.PerformAction.Response response = performService_.call(req);
			return StateConverter.msgToState(response.new_state);
		} catch (RosException e) {
			e.printStackTrace();
			return null;
		}
	}

	@Override
	public OOMDPState initializeEnvironment(List<OOMDPObjectState> objects) {
		InitializeEnvironment.Request req = new InitializeEnvironment.Request();
		req.object_states = StateConverter.objectsToMsgArray(objects);
		try {
			ros.pkg.oomdp_msgs.srv.InitializeEnvironment.Response response = initializeService_.call(req);
			return StateConverter.msgToState(response.start_state);
		} catch (RosException e) {
			e.printStackTrace();
			return null;
		}
	}

	@Override
	public OOMDPState computeRelations(List<OOMDPObjectState> objects) {
		// TODO: Call "environment/compute_relations"
		throw new RuntimeException("computeRelations is not yet supported");
	}
}
