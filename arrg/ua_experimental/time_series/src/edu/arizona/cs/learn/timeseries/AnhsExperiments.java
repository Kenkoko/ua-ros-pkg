package edu.arizona.cs.learn.timeseries;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizerStatistics;

public class AnhsExperiments {
	
	public static void main(String[] args) {

//		runDecompositionExperiment();
		runRecognitionExperiment();
//		runInformationDepthExperiment();
	}
	
	
	private static void runInformationDepthExperiment() {
		int[] pcts = { 80 }; //{ 95 };
		boolean[] prunes = { true };
		String prefix = "ww3d"; // "wes-pen";
		String[] activities = { "approach", "jump-over", "jump-on", "push" };  //{ "left", "right" }; //{"d", "l", "a", "h" };
		String testActivity = "jump-over"; //"d"; 
		SequenceType type = SequenceType.allen;
		Recognizer recognizer = Recognizer.cave;
		boolean optimizeRecognizers = true;


		for (boolean prune : prunes) {
			for (int i : pcts) {
				Experiments cv = new Experiments(0);
				cv.informationDepth(prefix, activities, testActivity, recognizer, type, 
						i, prune, false, optimizeRecognizers);
			}
		}
	}


	private static void runDecompositionExperiment() {
		try {
			int[] pcts = { 80 };
			boolean[] prunes = { true };
			int experimentsCount = 20;
			String prefix = "ww3d";
			String[] activities = { "jump-over" }; //, "jump-on", "left", "push", "right" };
			String subActivity = "approach";
			SequenceType type = SequenceType.allen;
			Recognizer recognizer = Recognizer.cave;
			boolean optimizeRecognizers = true;
			boolean composeGraphs = true;
			
			for (String activity : activities) {
				for (boolean prune : prunes) {
					for (int i : pcts) {
							
						Experiments cv = new Experiments(0);
						cv.decomposition(prefix, activity, subActivity,
								recognizer, type, i, prune, false,
								optimizeRecognizers, experimentsCount,
								composeGraphs);
						
						
					}
				}
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	private static void runRecognitionExperiment() {
		try {
			int experiments = 15;
			String prefix = "ww3d";
//			int[] folds = { 2, 3, 4, 5, 6, 7, 8, 9, 10 };
			int[] folds = { 6 };
			int[] pcts = { 80 };
			boolean[] prunes = { true };
			boolean[] optimizeRecognizers = { true };

			boolean setup = true;
			
			SequenceType type = SequenceType.allen;
			Recognizer recognizer = Recognizer.cave;
			boolean outputRecognizer = true;
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-hh-mm");
			String fileName = "data/recognizer-" + prefix + "-" +
					type + "-all-" + dateFormat.format(new Date()) + ".csv";
			
			BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
			out.write("experimentID,className,type,folds,recognizer,optimized,prune,minPct,precision,recall,f\n");
			
			for (int expID = 0; expID < experiments; expID++) {
				for (int f : folds) {

					if (setup) {
						Experiments.selectCrossValidation(prefix, f);
					}
						
					for (boolean prune : prunes) {
						
						if (setup) {
							Experiments.signatures(prefix, type, f, prune);
						}
						
						for (boolean optimize : optimizeRecognizers) {
							
							for (int i : pcts) {
								
								Experiments cv = new Experiments(f);
								Map<String, RecognizerStatistics> map = 
									cv.recognition(prefix, recognizer, type, i, prune,
											false, outputRecognizer, optimize);
								
								System.out.println("i=" + i + ", folds=" + f);
								
								for (String className : map.keySet()) {
									RecognizerStatistics rs = map.get(className);
									out.write(expID + "," 
											+ className + "," 
											+ type + "," 
											+ f + ","
											+ recognizer.name() + "," 
											+ ((optimize) ? "true" : "false") + ","
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
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
