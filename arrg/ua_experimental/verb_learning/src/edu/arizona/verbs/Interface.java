package edu.arizona.verbs;

import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;
import ros.pkg.oomdp_msgs.srv.PerformAction;
import ros.pkg.verb_learning.msg.VerbDescription;
import ros.pkg.verb_learning.msg.VerbInstance;
import ros.pkg.verb_learning.srv.ConstrainVerb;
import ros.pkg.verb_learning.srv.ForgetVerb;
import ros.pkg.verb_learning.srv.LoadVerbs;
import ros.pkg.verb_learning.srv.PerformVerb;
import ros.pkg.verb_learning.srv.LoadVerbs.Request;
import ros.pkg.verb_learning.srv.PlanVerb;
import ros.pkg.verb_learning.srv.UpdateVerb;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.verbs.environment.Environment;
import edu.arizona.verbs.environment.GazeboEnvironment;
import edu.arizona.verbs.mdp.OOMDPObjectState;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.util.TraceConverter;

public class Interface {
	private static Logger logger = Logger.getLogger(Interface.class);

	// This is for concatenation tests
	private static boolean HACKHACKHACK = false;
	
	// These are the verbs that we have learned!
	private static HashMap<String, Verb> verbs = new HashMap<String, Verb>();
	
	private static Environment currentEnvironment = null;
	public static Environment getCurrentEnvironment() { return currentEnvironment; }
	
	// Saves a heatmap to file. For reference, see Heatmaps.java
	// TODO: This should probably be somewhere else
//	public static void makeHeatmap(Verb verb, List<Interval> episode, int min) {
//		String f = verb.getVerbFolder() + "heatmap.png"; // Will overwrite earlier heatmaps, that's OK
//		HeatmapImage.makeHeatmap(f, verb.getSignature().signature(), min, episode, SequenceType.allen);
//	}

	public static Map<String, String> extractNameMap(VerbInstance vi) {
		if (vi.arguments.length == vi.bindings.length) {
			Map<String, String> nameMap = new HashMap<String, String>();
			for (int i = 0; i < vi.arguments.length; i++) {
				nameMap.put(vi.bindings[i], vi.arguments[i]);
			}
			return nameMap;
		} else {
			throw new RuntimeException("NAME MAP LENGTHS DO NOT MATCH IN VerbInstance");
		}
	}
	
	// Service Handlers
	
	static ServiceServer.Callback<UpdateVerb.Request, UpdateVerb.Response> updateVerb = 
		new ServiceServer.Callback<UpdateVerb.Request, UpdateVerb.Response>() {
			@Override
			public UpdateVerb.Response call(UpdateVerb.Request request) {
				logger.debug("BEGIN update_verb callback...");
				logger.debug("Received a trace with " + request.trace.length + " states for <" + request.verb + ">");
				
				String verbName = request.verb.verb;
				Map<String, String> nameMap = extractNameMap(request.verb);
				Instance instance = TraceConverter.convertTrace(StateConverter.msgArrayToStates(request.trace), verbName, nameMap);
				Verb verb = null;
				if (verbs.containsKey(verbName)) {
					verb = verbs.get(verbName); 
				} else {
					logger.info("VERB NOT FOUND: " + request.verb + ", creating it");
					verb = new Verb(verbName, request.verb.arguments);
					verbs.put(verbName, verb);
				}

				if (request.is_positive > 0) {
					verb.addPositiveInstance(instance);
				} else {
					verb.addNegativeInstance(instance);
				}
				
				logger.debug("... END update_verb callback.");
				return new UpdateVerb.Response();
			}
		};
	
	static ServiceServer.Callback<ForgetVerb.Request, ForgetVerb.Response> forgetVerb =
		new ServiceServer.Callback<ForgetVerb.Request, ForgetVerb.Response>() {
			@Override
			public ForgetVerb.Response call(ForgetVerb.Request request) {
				String verbName = request.verb.verb;
				if (verbs.containsKey(verbName)) {
					if (!HACKHACKHACK) {
						verbs.remove(verbName);
					}
				} 
				
				return new ForgetVerb.Response();
			}
		};
		
	static ServiceServer.Callback<LoadVerbs.Request, LoadVerbs.Response> loadVerbs =
		new ServiceServer.Callback<LoadVerbs.Request, LoadVerbs.Response>() {
			@Override
			public LoadVerbs.Response call(LoadVerbs.Request request) {
				LoadVerbs.Response resp = new LoadVerbs.Response();
				
				File verbDirectory = new File("verbs");
				FilenameFilter filter = new FilenameFilter() {
				    public boolean accept(File dir, String name) {
				    	return dir.isDirectory() && !name.startsWith(".");
				    }
				};
				
				Vector<VerbDescription> verbDescriptions = new Vector<VerbDescription>();
				String[] verbDirs = verbDirectory.list(filter);
				if (verbDirs == null) {
					logger.error("PROBLEM LOADING VERBS");
				} else {
					logger.debug("LOADING " + verbDirs.length + " SIGNATURES...");
				    for (String dir : verbDirs) {
				    	Vector<String> strings = new Vector<String>();
				    	Iterables.addAll(strings, Splitter.on(",").split(dir));
				    	
				    	Signature positiveSignature, negativeSignature;
				    	
				    	String signatureFile = "verbs/" + dir + "/signature.xml";
				    	File f = new File(signatureFile);
				    	if (f.exists()) {
				    		positiveSignature = Signature.fromXML(signatureFile);
				    	} else {
				    		positiveSignature = new Signature(strings.firstElement());
				    	}
				    	
				    	String negSigFile = "verbs/" + dir + "/neg-signature.xml";
				    	if (new File(negSigFile).exists()) {
				    		negativeSignature = Signature.fromXML(negSigFile);
				    	} else {
				    		negativeSignature = new Signature("non-" + strings.firstElement());
				    	}
				    	
				        Verb verb = new Verb(strings.firstElement(),
				        				     strings.subList(1, strings.size()).toArray(new String[0]),
				        				     positiveSignature, negativeSignature);
				        verbs.put(verb.getLexicalForm(), verb);
				        
				        VerbDescription desc = new VerbDescription();
				        desc.verb = verb.getLexicalForm();
				        desc.arguments = verb.getArgumentArray();
				        verbDescriptions.add(desc);
				    }
				}
				
				resp.verbs = verbDescriptions.toArray(new VerbDescription[0]);
				
				logger.debug("VERBS LOADED");
				
				return resp;
			}
		};
		
	static ServiceServer.Callback<ConstrainVerb.Request, ConstrainVerb.Response> constrainVerb = 
		new ServiceServer.Callback<ConstrainVerb.Request, ConstrainVerb.Response>() {
			@Override
			public ConstrainVerb.Response call(ConstrainVerb.Request request) {
				ConstrainVerb.Response resp = new ConstrainVerb.Response();
				
				Verb verb = verbs.get(request.verb);
				verb.addConstraint(Arrays.asList(request.banned_props));
				
				return resp;
			}
		};
		
	static ServiceServer.Callback<PerformVerb.Request, PerformVerb.Response> performVerb = 
		new ServiceServer.Callback<PerformVerb.Request, PerformVerb.Response>() {
		@Override
		public PerformVerb.Response call(PerformVerb.Request request) {
			if (verbs.containsKey(request.verb.verb)) {
				Verb verb = verbs.get(request.verb.verb);
				Map<String,String> argumentMap = extractNameMap(request.verb);
				return verb.perform(request.start_state, argumentMap, request.execution_limit);
			} else {
				System.err.println("Verb not found: " + request.verb.verb);
				return new PerformVerb.Response();
			}
		}
	};
		
	static ServiceServer.Callback<PerformAction.Request, PerformAction.Response> performAction = 
		new ServiceServer.Callback<PerformAction.Request, PerformAction.Response>() {
			@Override
			public PerformAction.Response call(PerformAction.Request request) {
				PerformAction.Response resp = new PerformAction.Response();
				
				OOMDPState performResult = currentEnvironment.performAction(request.action);
				resp.new_state = StateConverter.stateToMsg(performResult);
				
				return resp;
			}
		};
		
	static ServiceServer.Callback<InitializeEnvironment.Request, InitializeEnvironment.Response> initializeEnvironment = 
		new ServiceServer.Callback<InitializeEnvironment.Request, InitializeEnvironment.Response>() {
			@Override
			public InitializeEnvironment.Response call(InitializeEnvironment.Request request) {
				InitializeEnvironment.Response resp = new InitializeEnvironment.Response();
				
				Vector<OOMDPObjectState> states = new Vector<OOMDPObjectState>();
				for (MDPObjectState stateMsg : request.object_states) {
					states.add(StateConverter.msgToObjState(stateMsg));
				}
				OOMDPState initializeResult = currentEnvironment.initializeEnvironment(states);
				resp.start_state = StateConverter.stateToMsg(initializeResult);
				return resp;
			}
		};

	public static void testWW2D() {
		OOMDPObjectState obj1 = new OOMDPObjectState("agent1", "agent");
		obj1.setAttribute("x", "10");
		obj1.setAttribute("y", "10");
		obj1.setAttribute("angle", "0.00");
		obj1.setAttribute("shape-type", "circle");
		obj1.setAttribute("radius", "0.25");

		OOMDPObjectState obj2 = new OOMDPObjectState("agent2", "agent");
		obj2.setAttribute("x", "15");
		obj2.setAttribute("y", "15");
		obj2.setAttribute("angle", "0.00");
		obj2.setAttribute("shape-type", "circle");
		obj2.setAttribute("radius", "0.25");
		
		List<OOMDPObjectState> objects = new ArrayList<OOMDPObjectState>();
		objects.add(obj1);
		objects.add(obj2);
		
		OOMDPState state = currentEnvironment.initializeEnvironment(objects);

		System.out.println("Initialization complete...");
		System.out.println(state.toString());
		
		System.out.println("Printing actions...");
		System.out.println(currentEnvironment.getActions());
	}
		
	/**
	 * @param args
	 * @throws RosException
	 */
	public static void main(String[] args) throws RosException {
		// TODO: Should take the environment as an argument
		
		final Ros ros = Ros.getInstance();
		ros.init("verb_learning");
		NodeHandle nh = ros.createNodeHandle();

		// Gazebo
		currentEnvironment = new GazeboEnvironment();
		
		// Wubble World 2D
//		currentEnvironment = new WW2DEnvironment(true);
		
//		testWW2D();
		
		nh.advertiseService("verb_learning/load_verbs", new LoadVerbs(), loadVerbs);
		nh.advertiseService("verb_learning/forget_verb", new ForgetVerb(), forgetVerb);
		
		nh.advertiseService("verb_learning/update_verb", new UpdateVerb(), updateVerb);
		
		nh.advertiseService("verb_learning/perform_verb", new PerformVerb(), performVerb);

		// These are for the teacher to use, since we will now be embedding some of the environments
		nh.advertiseService("verb_learning/initialize_environment", new InitializeEnvironment(), initializeEnvironment);
		nh.advertiseService("verb_learning/perform_action", new PerformAction(), performAction);
		
		// This is old, should be removed
		nh.advertiseService("verb_learning/constrain_verb", new ConstrainVerb(), constrainVerb);
		
		logger.info("Initialization Complete, Services Advertised.");
		
		// This is for go-go testing, not normal operation
		if (HACKHACKHACK) {
			LoadVerbs.Request fake = new Request();
			loadVerbs.call(fake);
			Verb go = verbs.get("go");
			Verb go2 = verbs.get("go2");
			go.concatenate(go2, "go-through", new String[] {"thing", "waypoint", "goal"});
			verbs.put("go-through", go);
		}
		
		ros.spin();
	}
}
