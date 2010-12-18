package edu.arizona.verbs.environments;

import java.util.ArrayList;
import java.util.List;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;
import ros.pkg.oomdp_msgs.srv.PerformAction;
import ros.pkg.oomdp_msgs.srv.SimulateAction;
import ros.pkg.oomdp_msgs.srv.SimulateAction.Response;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;

public class GazeboEnvironment implements Environment {
	private NodeHandle nodeHandle_;
	private ros.ServiceClient<SimulateAction.Request, SimulateAction.Response, SimulateAction> simulateService_;
	private ros.ServiceClient<InitializeEnvironment.Request, InitializeEnvironment.Response, InitializeEnvironment> initializeService_;
	private ros.ServiceClient<PerformAction.Request, PerformAction.Response, PerformAction> performService_;
	
	private ArrayList<String> actions_ = null;
	
	public GazeboEnvironment() {
		try {
			Ros.getInstance().init("environment_interface");
		} catch (IllegalArgumentException e) {
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
		
		for (int i = 0; i < state.getObjectStates().size(); i++) {
			request.state.object_states.add(StateConverter.objStateToMsg(state.getObjectStates().get(i)));
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
		return actions_;
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
		req.object_states = StateConverter.objectsToMsgArrayList(objects);
		try {
			ros.pkg.oomdp_msgs.srv.InitializeEnvironment.Response response = initializeService_.call(req);
			actions_ = response.actions;
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

	@Override
	public OOMDPState simulateAction(String action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		
	}
}
