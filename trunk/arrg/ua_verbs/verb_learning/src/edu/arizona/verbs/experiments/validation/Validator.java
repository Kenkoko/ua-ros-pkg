package edu.arizona.verbs.experiments.validation;

import java.util.List;

import com.google.common.collect.Lists;

import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.environments.Simulators;
import edu.arizona.verbs.experiments.Experimenter;
import edu.arizona.verbs.experiments.Scenario;
import edu.arizona.verbs.experiments.evaluation.Label;
import edu.arizona.verbs.experiments.evaluation.Labeler;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;

public class Validator {

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		String env = "ww2d";
		String verb = "avoid";
		
		List<Scenario> scenarios = Experimenter.loadScenarios(env, verb);

		WW2DEnvironment.visualize = true;
		Environment environment = Simulators.valueOf(env).create();
		
		List<Label> labels = Lists.newArrayList();
		
		for (Scenario scenario : scenarios) {
			List<OOMDPState> demonstration = Experimenter.demonstrate(scenario, environment);
			Label label = Labeler.valueOf(verb).label(demonstration);
			labels.add(label);
		}

		System.out.println("\n\n\n");
		System.out.println("**********************************");
		System.out.println("FINAL REPORT: ");
		int i = 1;
		for (Label label : labels) {
			System.out.println((i++) + "\t" + label.toString());
		}
	}

}
