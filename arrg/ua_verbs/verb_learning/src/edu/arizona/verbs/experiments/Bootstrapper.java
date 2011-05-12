package edu.arizona.verbs.experiments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import ros.pkg.verb_learning.srv.PerformVerb.Response;

import au.com.bytecode.opencsv.CSVWriter;

import com.google.common.collect.Lists;

import edu.arizona.verbs.environments.Simulators;
import edu.arizona.verbs.environments.ww2d.WW2DEnvironment;
import edu.arizona.verbs.experiments.label.Label;
import edu.arizona.verbs.experiments.label.Labelers;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;
import edu.arizona.verbs.verb.VerbBinding;
import edu.arizona.verbs.verb.Verbs;
import edu.arizona.verbs.verb.vfsm.FSMVerb;
import edu.arizona.verbs.verb.vfsm.SequentialVerb;

public class Bootstrapper {

	
	
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		String env = "ww2d";
//		String env = "gazebo";
		
		String verb = "go_via";
		ArrayList<String> arguments = Lists.newArrayList("agent", "waypoint", "place");

		String model = "corepath";
		
		int numTrajectories = 15;
		
//		WW2DEnvironment.visualize = true;
		Environment environment = Simulators.valueOf(env).create();
		
		List<Scenario> baseScenarios = Experimenter.loadScenarios(env, "go");
		// Pre-cache the teacher demonstrations
		List<List<OOMDPState>> baseDemonstrations = Lists.newArrayList();
		for (Scenario scenario : baseScenarios) {
			baseDemonstrations.add(Experimenter.demonstrate(scenario, environment));
		}
		
		// Load the scenario file
		List<Scenario> scenarios = Experimenter.loadScenarios(env, verb);
		// Pre-cache the teacher demonstrations
		List<List<OOMDPState>> demonstrations = Lists.newArrayList();
		for (Scenario scenario : scenarios) {
			demonstrations.add(Experimenter.demonstrate(scenario, environment));
		}

		File dataFolder = new File(System.getProperty("user.dir") + "/data/csv/" + env + "_" + verb + "_" + "bootstrapped");
		if (!dataFolder.isDirectory()) {
			dataFolder.mkdir();
		}
		
		// Create the CSV file
		String csvString = String.valueOf(System.currentTimeMillis());
		CSVWriter writer = new CSVWriter(new FileWriter(dataFolder.getAbsolutePath() + "/" + csvString + ".csv"));
		
		List<List<Integer>> orders = Lists.newArrayList();
		
		Random random = new Random();
		
		for (int i = 0; i < numTrajectories; i++) {
			
			// Make the base verb afresh
			Verb baseVerb = Verbs.valueOf(model).create("go", Lists.newArrayList("agent", "place"));
			
			// Make the sequential verb
			VerbBinding go1 = new VerbBinding();
			go1.verb = (FSMVerb) baseVerb;
			go1.binding = new LinkedHashMap<String, String>();
			go1.binding.put("agent", "agent");
			go1.binding.put("place", "waypoint");
			
			VerbBinding go2 = new VerbBinding();
			go2.verb = (FSMVerb) baseVerb;
			go2.binding = new LinkedHashMap<String, String>();
			go2.binding.put("agent", "agent");
			go2.binding.put("place", "place");
				
			SequentialVerb v = new SequentialVerb(verb, arguments, Lists.newArrayList(go1, go2));
			
			List<Integer> randomized = Lists.newArrayList();
			for (int n = 0; n < scenarios.size(); n++) {
				randomized.add(n);
			}
			Collections.shuffle(randomized);
			orders.add(randomized);
			
			// A copy so we can remove from it
			List<List<OOMDPState>> baseDemoCopy = Lists.newArrayList(baseDemonstrations);
			
			int current = 0;
			String[] labels = new String[randomized.size()];
			String[] times = new String[randomized.size()];
			
			for (int index : randomized) {
				
				//////////////////////////////////////////////////////
				// Step 1: Agent performs the verb
				OOMDPState startState = environment.initializeEnvironment(scenarios.get(index).start);
				Response response = v.perform(environment, startState, 30);
				
				List<OOMDPState> studentTrace = StateConverter.msgArrayListToStates(response.trace);
				
				//////////////////////////////////////////////////////
				// Step 2: Teacher labels performance - TODO: Do we want learning in here for this?
				Label label = Labelers.valueOf(env).getLabeler(verb).label(studentTrace);
				System.out.println(label);
				switch (label) {
				case Success:
					v.addPositiveInstance(studentTrace);
//					correct[current] = true;
					break;
				case Violation:
					v.addNegativeInstance(studentTrace);
					break;
				}
				
				labels[current] = label.toString();
				times[current] = String.valueOf(response.planning_time);
				
				//////////////////////////////////////////////////////
				// Step 3: Teacher demonstrates
//				v.addPositiveInstance(demonstrations.get(index));
				if (!baseDemoCopy.isEmpty()) {
					List<OOMDPState> baseDemo = baseDemoCopy.remove(random.nextInt(baseDemoCopy.size()));
					baseVerb.addPositiveInstance(baseDemo);
					v.makeFSM();
				}
				
				current++;
			}
			
			// Write the CSV file
			writer.writeNext(labels);
			writer.writeNext(times);
			writer.flush();
//			System.out.println(Arrays.toString(labels));
//			System.out.println();
			
		}
		
		// Close the CSV file
		writer.close();
		
		System.out.println("\n\nPERMUTATIONS");
		for (List<Integer> order : orders) {
			System.out.println(order);
		}
	}

}
