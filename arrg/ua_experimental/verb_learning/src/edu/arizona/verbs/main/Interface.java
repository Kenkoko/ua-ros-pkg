package edu.arizona.verbs.main;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;

import org.apache.log4j.Logger;
import org.yaml.snakeyaml.Yaml;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.ServiceServer.Callback;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;
import ros.pkg.oomdp_msgs.srv.PerformAction;
import ros.pkg.verb_learning.msg.VerbDescription;
import ros.pkg.verb_learning.msg.VerbInstance;
import ros.pkg.verb_learning.srv.DefineSequentialVerb;
import ros.pkg.verb_learning.srv.DefineSequentialVerb.Response;
import ros.pkg.verb_learning.srv.ForgetVerb;
import ros.pkg.verb_learning.srv.LoadVerbs;
import ros.pkg.verb_learning.srv.PerformVerb;
import ros.pkg.verb_learning.srv.UpdateVerb;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.environments.GazeboEnvironment;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.AtomicVerb;
import edu.arizona.verbs.verb.SequentialVerb;
import edu.arizona.verbs.verb.Verb;
import edu.arizona.verbs.verb.VerbBinding;

public class Interface {
	private static Logger logger = Logger.getLogger(Interface.class);

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

	// Maps: Binding -> Argument
	public static Map<String, String> extractReverseNameMap(VerbInstance vi) {
		if (vi.arguments.size() == vi.bindings.size()) {
			Map<String, String> nameMap = new HashMap<String, String>();
			for (int i = 0; i < vi.arguments.size(); i++) {
				nameMap.put(vi.bindings.get(i), vi.arguments.get(i));
			}
			return nameMap;
		} else {
			throw new RuntimeException("NAME MAP LENGTHS DO NOT MATCH IN VerbInstance");
		}
	}
	
	// Maps: Argument -> Binding
	public static LinkedHashMap<String, String> extractNameMap(VerbInstance vi) {
		if (vi.arguments.size() == vi.bindings.size()) {
			LinkedHashMap<String, String> nameMap = new LinkedHashMap<String, String>();
			for (int i = 0; i < vi.arguments.size(); i++) {
				nameMap.put(vi.arguments.get(i), vi.bindings.get(i));
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
				logger.debug("Received a trace with " + request.trace.size() + " states for <" + request.verb + ">");
				
				String verbName = request.verb.verb;
				Map<String, String> nameMap = extractReverseNameMap(request.verb);
				Instance instance = StateConverter.convertTrace(StateConverter.msgArrayToStates(request.trace), verbName, nameMap);
				Verb verb = null;
				if (verbs.containsKey(verbName)) {
					verb = verbs.get(verbName); 
				} else {
					logger.info("VERB NOT FOUND: " + request.verb + ", creating it");
					verb = new AtomicVerb(verbName, request.verb.arguments);
					verbs.put(verbName, verb);
				}

				if (request.is_positive) {
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
					Verb verb = verbs.get(request.verb.verb);
					verb.forgetInstances();
				} 
				
				return new ForgetVerb.Response();
			}
		};
		
	static ServiceServer.Callback<ForgetVerb.Request, ForgetVerb.Response> deleteVerb =
		new ServiceServer.Callback<ForgetVerb.Request, ForgetVerb.Response>() {
			@Override
			public ForgetVerb.Response call(ForgetVerb.Request request) {
				String verbName = request.verb.verb;
				if (verbs.containsKey(verbName)) {
					verbs.remove(verbName);
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
				
				String[] verbDirs = verbDirectory.list(filter);
				if (verbDirs == null) {
					logger.error("PROBLEM LOADING VERBS");
				} else {
					logger.debug("LOADING " + verbDirs.length + " SIGNATURES...");
					List<String> compositeVerbs = new Vector<String>();
					
					// First pass loads all the atomic verbs, marks the others to load later
				    for (String dir : verbDirs) {
				    	ArrayList<String> strings = new ArrayList<String>();
				    	Iterables.addAll(strings, Splitter.on(",").split(dir));
				    	
				    	File seq = new File("verbs/" + dir + "/sequential.yaml");
				    	if (seq.exists()) { // The verb is a sequential verb
				    		compositeVerbs.add(dir); // We will load these only after all the atomic verbs
				    		
				    	} else { // The verb is an atomic verb
				    		Signature positiveSignature, negativeSignature;
				    		
				    		String signatureFile = "verbs/" + dir + "/signature.xml";
				    		File f = new File(signatureFile);
				    		if (f.exists()) {
				    			positiveSignature = Signature.fromXML(signatureFile);
				    		} else {
				    			positiveSignature = new Signature(strings.get(0));
				    		}
				    		
				    		String negSigFile = "verbs/" + dir + "/neg-signature.xml";
				    		if (new File(negSigFile).exists()) {
				    			negativeSignature = Signature.fromXML(negSigFile);
				    		} else {
				    			negativeSignature = new Signature("non-" + strings.get(0));
				    		}
				    		
				    		AtomicVerb verb = new AtomicVerb(strings.get(0),
				    										 strings.subList(1, strings.size()),
				    										 positiveSignature, negativeSignature);
				    		verbs.put(verb.getLexicalForm(), verb);
				    	}
				    }
				    	
				    // Now we load the composite verbs
			    	for (String compDir : compositeVerbs) {
			    		Vector<String> strings = new Vector<String>();
				    	Iterables.addAll(strings, Splitter.on(",").split(compDir));
			    		
				    	try {
				    		File seq = new File("verbs/" + compDir + "/sequential.yaml");
							Yaml yaml = new Yaml();
							@SuppressWarnings("rawtypes")
							List yamlList = (List) yaml.load(new FileInputStream(seq));

							List<VerbBinding> subverbs = new Vector<VerbBinding>();
							
							for (Object o : yamlList) {
								@SuppressWarnings("unchecked")
								Map<String,Map<String,String>> map = (Map<String, Map<String, String>>) o;
								Entry<String, Map<String, String>> entry = Iterables.getOnlyElement(map.entrySet());
								String verbFolder = entry.getKey();
								LinkedHashMap<String,String> binding = (LinkedHashMap<String, String>) entry.getValue();
								
								VerbBinding vb = new VerbBinding();
								vb.verb = verbs.get(Iterables.getFirst(Splitter.on(",").split(verbFolder), null));
								vb.binding = binding;
								subverbs.add(vb);
							}
							
							// TODO: Need to load the signatures, etc. if they exist
							Signature positiveSignature, negativeSignature;
							String signatureFile = "verbs/" + compDir + "/signature.xml";
				    		File f = new File(signatureFile);
				    		if (f.exists()) {
				    			positiveSignature = Signature.fromXML(signatureFile);
				    		} else {
				    			positiveSignature = new Signature(strings.firstElement());
				    		}
				    		
				    		String negSigFile = "verbs/" + compDir + "/neg-signature.xml";
				    		if (new File(negSigFile).exists()) {
				    			negativeSignature = Signature.fromXML(negSigFile);
				    		} else {
				    			negativeSignature = new Signature("non-" + strings.firstElement());
				    		}
							
							ArrayList<String> split = Lists.newArrayList(Splitter.on(",").split(compDir));
							SequentialVerb sv = new SequentialVerb(split.get(0), split.subList(1, split.size()), subverbs);
							sv.setPositiveSignature(positiveSignature);
							sv.setNegativeSignature(negativeSignature);
							verbs.put(split.get(0), sv);
							
						} catch (FileNotFoundException e) {
							e.printStackTrace();
						}
				    }
				}
				
				for (Verb verb : verbs.values()) {
					VerbDescription desc = new VerbDescription();
		    		desc.verb = verb.getLexicalForm();
		    		desc.arguments = verb.getArguments();
		    		resp.verbs.add(desc);
				}
				
				System.out.println(verbs);
				
				logger.debug("VERBS LOADED");
				
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
				
				Verb remapped = verb.remap(argumentMap);
//				return verb.perform(request.start_state, argumentMap, request.execution_limit);
				return remapped.perform(request.start_state, request.execution_limit);
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
		
	static Callback<DefineSequentialVerb.Request, DefineSequentialVerb.Response> defineVerb =
		new Callback<DefineSequentialVerb.Request, DefineSequentialVerb.Response>() {
			@Override
			public Response call(DefineSequentialVerb.Request request) {
				Vector<VerbBinding> subverbs = new Vector<VerbBinding>();
				for (VerbInstance vi : request.subverbs) {
					VerbBinding vb = new VerbBinding();
					vb.verb = verbs.get(vi.verb); // TODO: Need some kind of a check here
					vb.binding = extractNameMap(vi); // TODO: Confirm this is the right direction
				
					subverbs.add(vb);
				}
				SequentialVerb verb = new SequentialVerb(request.verb, request.arguments, subverbs);
				verbs.put(request.verb, verb);
				
				return new Response();
			}
		};

	/**
	 * @param args
	 * @throws RosException
	 */
	public static void main(String[] args) throws RosException {
		// TODO: Should take the environment (and preferred planner?) as an argument
		
		final Ros ros = Ros.getInstance();
		ros.init("verb_learning");
		NodeHandle nh = ros.createNodeHandle();

		// Gazebo
		currentEnvironment = new GazeboEnvironment();
		
		// Wubble World 2D
//		currentEnvironment = new WW2DEnvironment(true);
		
		nh.advertiseService("verb_learning/load_verbs", new LoadVerbs(), loadVerbs);
		nh.advertiseService("verb_learning/forget_verb", new ForgetVerb(), forgetVerb);
		
		nh.advertiseService("verb_learning/update_verb", new UpdateVerb(), updateVerb);
		nh.advertiseService("verb_learning/define_verb", new DefineSequentialVerb(), defineVerb);
		
		nh.advertiseService("verb_learning/perform_verb", new PerformVerb(), performVerb);

		// These are for the teacher to use, since we will now be embedding some of the environments
		nh.advertiseService("verb_learning/initialize_environment", new InitializeEnvironment(), initializeEnvironment);
		nh.advertiseService("verb_learning/perform_action", new PerformAction(), performAction);
		
		logger.info("Initialization Complete, Services Advertised.");
		
		ros.spin();
	}
}
