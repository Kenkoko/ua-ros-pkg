package edu.arizona.verbs;

import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;
import ros.pkg.oomdp_msgs.srv.PerformAction;
import ros.pkg.time_series.msg.Episode;
import ros.pkg.verb_learning.msg.VerbDescription;
import ros.pkg.verb_learning.srv.ConstrainVerb;
import ros.pkg.verb_learning.srv.FindSignature;
import ros.pkg.verb_learning.srv.ForgetVerb;
import ros.pkg.verb_learning.srv.LoadVerbs;
import ros.pkg.verb_learning.srv.LoadVerbs.Request;
import ros.pkg.verb_learning.srv.PlanVerb;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.cs.learn.algorithm.alignment.factory.SequenceFactory;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.heatmap.HeatmapImage;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.verbs.environment.Environment;
import edu.arizona.verbs.environment.GazeboEnvironment;
import edu.arizona.verbs.mdp.OOMDPObjectState;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.StateConverter;

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
	public static void makeHeatmap(Verb verb, List<Interval> episode, int min) {
		String f = verb.getVerbFolder() + "heatmap.png"; // Will overwrite earlier heatmaps, that's OK
		HeatmapImage.makeHeatmap(f, verb.getSignature().signature(), min, episode, SequenceType.allen);
	}
	
	// Utility functions for converting ROS message types to the appropriate Java classes
	public static List<Interval> getIntervals(Episode episode) {
		logger.debug("Getting intervals from episode...");
		List<Interval> intervals = new Vector<Interval>();
		for (int i = 0; i < episode.intervals.length; i++) {
			ros.pkg.time_series.msg.Interval interval = episode.intervals[i];
			logger.debug(interval.proposition);
			intervals.add(Interval.make(interval.proposition.toString(), interval.start, interval.end));
		}
		logger.debug("Got em.");
		return intervals;
	}
	
	public static Instance getInstance(Episode e, String name, int id) {
		List<Interval> intervals = getIntervals(e);		
		logger.debug(intervals.size());
		Collections.sort(intervals, Interval.eff);
		for (Interval i : intervals) {
			i.episode = id;
			i.file = "test";
			logger.debug(i);
		}
		logger.debug("Finished with " + intervals.size() + " intervals, converting to instance...");
		Instance result = new Instance(name, id, SequenceFactory.allenSequence(intervals));
		
		// TODO: SequenceFactory is in the time_series JAR! Not the src, which is probably a problem
		
		return result;
	}
	
	public static List<Instance> getSequences(Episode[] episodes, String name) {
		List<Instance> results = new ArrayList<Instance>();

		for (int i = 0; i < episodes.length; i++) {
			logger.debug("Converting episode " + (i+1) + " of " + episodes.length + " (" + episodes[i].intervals.length + " intervals)");
//			System.out.println(episodes[i].intervals[0]);
			results.add(getInstance(episodes[i], name, i));
		}
		
		return results;
	}
	
	// Service Handlers
	
	static ServiceServer.Callback<FindSignature.Request, FindSignature.Response> positiveUpdate = 
		new ServiceServer.Callback<FindSignature.Request, FindSignature.Response>() {
			public FindSignature.Response call(FindSignature.Request request) {
				
				FindSignature.Response res = new FindSignature.Response();
				logger.debug("BEGIN update_signature callback...");

				logger.debug("Received " + request.episodes.length + " episodes for <" + request.verb + ">");

				String verbName = request.verb.verb;
				if (verbs.containsKey(verbName)) {
					List<Instance> instances = getSequences(request.episodes, verbName);

					Verb verb = verbs.get(verbName); // VERB VERB VERB!
					for (Instance instance : instances) {
						verb.addPositiveInstance(instance);
					}
				} else {
					logger.info("VERB NOT FOUND: " + request.verb + ", creating it");
					List<Instance> instances = getSequences(request.episodes, verbName);
					if (verbs.containsKey(verbName)) {
						logger.info("Overwriting old data for verb: " + verbName);
					} 
					Verb verb = new Verb(verbName, request.verb.arguments);
					verb.addPositiveInstances(instances);
					verbs.put(verbName, verb);
				}
				
				logger.debug("... END update_signature callback.");
				return res;
			}
		};
	
	static ServiceServer.Callback<FindSignature.Request, FindSignature.Response> negativeUpdate = 
		new ServiceServer.Callback<FindSignature.Request, FindSignature.Response>() {
			public FindSignature.Response call(FindSignature.Request request) {
				
				FindSignature.Response res = new FindSignature.Response();
				logger.debug("BEGIN negative_update callback...");

				logger.debug("Received " + request.episodes.length + " negative episodes for <" + request.verb + ">");

				String verbName = request.verb.verb;
				if (verbs.containsKey(verbName)) {
					List<Instance> instances = getSequences(request.episodes, verbName);

					Verb verb = verbs.get(verbName); // VERB VERB VERB!
					for (Instance instance : instances) {
						verb.addNegativeInstance(instance);
					}
				} else {
					logger.info("VERB NOT FOUND: " + request.verb + ", creating it");
					List<Instance> instances = getSequences(request.episodes, verbName);
					if (verbs.containsKey(verbName)) {
						logger.info("Overwriting old data for verb: " + verbName);
					} 
					Verb verb = new Verb(verbName, request.verb.arguments);
					for (Instance instance : instances) {
						verb.addNegativeInstance(instance);
					}
					verbs.put(verbName, verb);
				}
				
				logger.debug("... END update_signature callback.");
				return res;
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
		
	// TODO: Rename to planVerb
	static ServiceServer.Callback<PlanVerb.Request, PlanVerb.Response> planVerb = 
		new ServiceServer.Callback<PlanVerb.Request, PlanVerb.Response>() {
		@Override
		public PlanVerb.Response call(PlanVerb.Request request) {
			if (verbs.containsKey(request.verb.verb)) {
				Verb verb = verbs.get(request.verb.verb);
				HashMap<String,String> argumentMap = new HashMap<String, String>();
				for (int i = 0; i < request.verb.arguments.length; i++) {
					argumentMap.put(request.verb.bindings[i], request.verb.arguments[i]);
				}
				return verb.planVerb(request.start_state, argumentMap);
			} else {
				System.err.println("Verb not found: " + request.verb.verb);
				return new PlanVerb.Response();
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
		
		// TODO LATER: Rename FindSignature to UpdateVerb
		nh.advertiseService("verb_learning/positive_update", new FindSignature(), positiveUpdate);
		nh.advertiseService("verb_learning/negative_update", new FindSignature(), negativeUpdate);
		
		nh.advertiseService("verb_learning/plan_verb", new PlanVerb(), planVerb);

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
