package edu.arizona.verbs.verb.vfsm;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.msg.VerbDescription;
import ros.pkg.verb_learning.srv.PerformVerb;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;

import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.main.Interface;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.fsm.Planners;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.learner.VFSMLearner;

public abstract class AbstractVerb implements FSMVerb {
	private static Logger logger = Logger.getLogger(AbstractVerb.class);
	
	protected String lexicalForm_;
	protected ArrayList<String> arguments_;
	
	AbstractVerb baseVerb_ = null;
	
	VFSMLearner posLearner_ = Interface.getVFSMLearner();
	VFSMLearner negLearner_ = Interface.getVFSMLearner();
	
	protected VerbFSM posFSM_ = null;
	protected VerbFSM negFSM_ = null;
	
	@Override
	public void addPositiveInstance(List<OOMDPState> trace) {
		posLearner_.addTrace(trace);
		postInstance();
	}
	
	@Override
	public void addNegativeInstance(List<OOMDPState> trace) {
		negLearner_.addTrace(trace);
		postInstance();
	}

	@Override
	public void addPositiveInstances(List<List<OOMDPState>> traces) {
		for (List<OOMDPState> trace : traces) {
			posLearner_.addTrace(trace);
		}
		postInstance();
	}
	
	@Override
	public void addNegativeInstances(List<List<OOMDPState>> traces) {
		for (List<OOMDPState> trace : traces) {
			negLearner_.addTrace(trace);
		}
		postInstance();
	}
	
	@Override
	public void forgetInstances() {
		posLearner_.forgetTraces();
		negLearner_.forgetTraces();
		posFSM_ = null;
		negFSM_ = null;
	}
	
	abstract void postInstance();

	/* Getters, etc. */
	
	@Override
	public String getLexicalForm() {
		return lexicalForm_;
	}

	@Override
	public String[] getArgumentArray() {
		return arguments_.toArray(new String[0]);
	}
	
	@Override
	public ArrayList<String> getArguments() {
		return arguments_;
	}
	
	@Override
	public VerbFSM getPositiveFSM() {
		return posFSM_;
	}

	@Override
	public VerbFSM getNegativeFSM() {
		return negFSM_;
	}
	
	@Override
	public boolean hasPositiveFSM() {
		return posFSM_ != null;
	}

	@Override
	public boolean hasNegativeFSM() {
		return negFSM_ != null;
	}
	
	@Override
	public boolean isReady() {
		return hasPositiveFSM();
	}

	/* Folders and ROS */
	
	void makeVerbFolder() {
		File verbFolder = new File(getVerbFolder());
		
		if (!verbFolder.exists()) {
			verbFolder.mkdirs();
		}
	}
	
	public String getVerbFolder() {
		return "verbs/" + getIdentifierString() + "/";
	}
	
	public VerbDescription makeVerbDescription() {
		VerbDescription desc = new VerbDescription();
		desc.verb = lexicalForm_;
		desc.arguments = arguments_;
		return desc;
	}

	@Override
	public String getIdentifierString() {
		return Joiner.on(",").join(Lists.asList(lexicalForm_, arguments_.toArray(new String[0])));
	}
	
	public AbstractVerb getBaseVerb() {
		if (baseVerb_ == null) {
			return this;
		} else {
			return baseVerb_;
		}
	}
	
	@Override
	public VerbState getStartState() {
		return new VerbState(posFSM_.getGlobalStartState(), negFSM_.getGlobalStartState());
	}
	
	@Override
	public PerformVerb.Response perform(MDPState startState, int executionLimit) {
		PerformVerb.Response response = new PerformVerb.Response();
		
		if (!hasPositiveFSM()) {
			return response; // Automatic failure
		}
		
		Planner planner = Planners.getPlanner(this, executionLimit);
		
		// MDP starts at the given start state
		OOMDPState mdpState = StateConverter.msgToState(startState);
		
		// Create initial verb state (transition on initial relations if possible
		VerbState verbState = getStartState();
		verbState = fsmTransition(verbState, mdpState.getActiveRelations());
		
		// Dummy policy which triggers immediate planning
		Policy policy = new Policy(PolicyType.Replan);
		
		long totalPlanningTime = 0;
		boolean executionSuccess = false;
		
		// This is for debugging
		ArrayList<VerbState> verbStates = new ArrayList<VerbState>();
		verbStates.add(verbState);
		
		// Main loop
		int numSteps = 0;
		while (numSteps < executionLimit) {
			// Add the current state to the trace
			response.trace.add(StateConverter.stateToMsg(mdpState));
			
			if (verbState.isGoodTerminal()) { // In case this happens unexpectedly
				System.out.println("------------ It is finished!");
				System.out.println(mdpState.getActiveRelations());
				response.actions.add(Action.TERMINATE);
				executionSuccess = true;
				break;
			}
			
			String action = policy.getAction(mdpState, verbState); 
			if (Action.REPLAN.equals(action)) { // This means we "fell off" the planned path
				logger.info("Fell off garden path, replanning...");
				planner.setMaxDepth(executionLimit - numSteps); // Needed by UCT
				PlanningReport report = planner.runAlgorithm(mdpState, verbState); // re-plan
				totalPlanningTime += report.getElaspedTime();
				
				if (totalPlanningTime > 1*60*1000) { // If we've spent over 2 minutes planning, can it
					action = Action.TERMINATE;
					executionSuccess = false;
					break;
				} else {
					policy = report.getPolicy();
					policy.print();
					
					action = policy.getAction(mdpState, verbState);
				}
			}
			
			// Record the action that we performed (to make validation easier)
			System.out.println("---> " + action);
			response.actions.add(action);
			
			if (Action.TERMINATE.equals(action)) {
				executionSuccess = (numSteps > 0); // If we immediately terminated, that's a failure
				break;
			} else {
				// Update to the new states
				mdpState = Interface.getCurrentEnvironment().performAction(action);
				verbState = fsmTransition(verbState, mdpState.getActiveRelations());
				
				if (Interface.getCurrentEnvironment() instanceof WW2DEnvironment) {
					for (OOMDPObjectState os : mdpState.getObjectStates()) {
						if (os.getValue("x").startsWith("-")) {
							System.out.println(os);
							System.err.println("SIMULATOR BLEW UP, REDOING THE WHOLE SHIZNIT");
							return perform(startState, executionLimit);
						}
					}
				}
				
				verbStates.add(verbState);
				
				System.out.println("****** STATE after " + numSteps + " steps: " + 
								   verbState + " " + mdpState);
				"a".length();
			}			
			
			numSteps++;
		}
		
		logger.info("Execution complete.");
		
		// What exactly does this mean? That we terminated "voluntarily"?
		response.execution_success = executionSuccess;
		response.execution_length = numSteps;
		response.planning_time = totalPlanningTime;
		
		logger.info("VERB STATES: ");
		int i = 0;
		for (VerbState vs : verbStates) {
			logger.debug(i + ": " + vs);
			i++;
		}
		
		return response;
	}
}
