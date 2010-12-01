package edu.arizona.cs.learn.timeseries;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizerStatistics;
import edu.arizona.cs.learn.util.SequenceType;

public class AnhsExperiments {
	
	public static void main(String[] args) {

		runDecompositionExperiment();
//		runRecognitionExperiment();
	}


	private static void runDecompositionExperiment() {
		try {
			int[] pcts = { 80 };
			boolean[] prunes = { true };
			int experimentsCount = 20;
			String prefix = "ww3d";
			String[] activities = { "jump-on", "jump-over", "left", "push", "right" };
			String subActivity = "approach";
			SequenceType type = SequenceType.allen;
			Recognizer recognizer = Recognizer.cave;
			boolean optimizeRecognizers = true;
			
			for (String activity : activities) {
				for (boolean prune : prunes) {
					for (int i : pcts) {
							
						Experiments cv = new Experiments(0);
						cv.decomposition(prefix, activity, subActivity,
								recognizer, type, i, prune, false,
								optimizeRecognizers, experimentsCount);
						
						
					}
				}
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	private static void runRecognitionExperiment() {
		try {
			int[] folds = { 6 };
			int[] pcts = { 80 };
			boolean[] prunes = { true , false };
			int experiments = 1; //20;
			String prefix = "global-ww2d";
			SequenceType type = SequenceType.allen;
			Recognizer recognizer = Recognizer.cave;
			boolean optimizeRecognizers = false;
			
			boolean setup = false;
			boolean outputRecognizer = true;
			
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-hh-mm");
			String fileName = "data/recognizer-" + prefix + "-" +
					type + "-all-" + dateFormat.format(new Date()) + ".csv";
			
			BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
			out.write("experimentID,className,type,fold,recognizer,optimized,prune,minPct,precision,recall,f\n");
			
			for (int expID = 0; expID < experiments; expID++) {
				for (boolean prune : prunes) {
					for (int i : pcts) {
						for (int fold : folds) {
		
							if (setup) {
								Experiments.selectCrossValidation(prefix, fold);
								Experiments.signatures(prefix, type, fold, prune);
							}
							
							Experiments cv = new Experiments(fold);
							Map<String, RecognizerStatistics> map = 
								cv.recognition(prefix, recognizer, type, i, prune,
										false, outputRecognizer, optimizeRecognizers);
							
							System.out.println("i=" + i + ", fold=" + fold);
							
							for (String className : map.keySet()) {
								RecognizerStatistics rs = map.get(className);
								out.write(expID + "," 
										+ className + "," 
										+ type + "," 
										+ fold + ","
										+ recognizer.name() + "," 
										+ ((optimizeRecognizers) ? "true" : "false") + ","
										+ ((prune) ? "true" : "false") + ","
										+ i + ","
										+ rs.precision() + ","
										+ rs.recall() + ","
										+ rs.fscore() + "\n");
							}
						}
					}
				}
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
