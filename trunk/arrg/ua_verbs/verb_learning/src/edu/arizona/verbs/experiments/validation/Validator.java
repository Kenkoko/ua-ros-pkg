package edu.arizona.verbs.experiments.validation;

import java.io.FileNotFoundException;
import java.util.Collections;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

import com.google.common.collect.Lists;

import edu.arizona.verbs.environments.Simulators;
import edu.arizona.verbs.environments.ww2d.WW2DEnvironment;
import edu.arizona.verbs.experiments.Experimenter;
import edu.arizona.verbs.experiments.Scenario;
import edu.arizona.verbs.experiments.label.Label;
import edu.arizona.verbs.experiments.label.Labelers;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;

public class Validator {

	public static void validate(Environment environment, String verb) throws FileNotFoundException {
		List<Scenario> scenarios = Experimenter.loadScenarios(environment.getNameString(), verb);

		List<Label> labels = Lists.newArrayList();
		
		int j = 1;
		for (Scenario scenario : scenarios) {
			List<OOMDPState> demonstration = Experimenter.demonstrate(scenario, environment);
			Label label = Labelers.valueOf(environment.getNameString()).getLabeler(verb).label(demonstration);
			labels.add(label);
			System.out.println((j++) + "\t" + label.toString());
		}

		System.out.println("\n\n\n");
		System.out.println("**********************************");
		System.out.println("FINAL REPORT: ");
		
		int i = 1;
		for (Label label : labels) {
			System.out.println((i++) + "\t" + label.toString());
		}
	}
	
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {

		// Turn off all logging from Log4j
		List<Logger> loggers = Collections.<Logger>list(LogManager.getCurrentLoggers());
		loggers.add(LogManager.getRootLogger());
		for ( Logger logger : loggers ) {
		    logger.setLevel(Level.FATAL);
		}

		
		WW2DEnvironment.visualize = true;
		Environment env = Simulators.valueOf("ww2d").create();
		
		validate(env, "go");
		validate(env, "avoid");
	}

}
