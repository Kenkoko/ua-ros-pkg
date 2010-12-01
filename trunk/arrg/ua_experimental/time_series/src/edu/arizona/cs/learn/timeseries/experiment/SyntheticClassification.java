package edu.arizona.cs.learn.timeseries.experiment;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

import edu.arizona.cs.learn.timeseries.Experiments;
import edu.arizona.cs.learn.timeseries.prep.SymbolicData;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

// Some things do not change
//    The number of examples... 60
//    The number of folds....   3

public class SyntheticClassification {
	public static final int FOLDS = 3;
	public static final int N = 60;

	public static void main(String[] args) { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;

		initialize();
		generateClass1();
	}
	
	/**
	 * Called once... it sets up the cross-validation
	 * folders and randomly shuffles the episodes into
	 * test sets.
	 * 
	 * After that it generates all of the examples for
	 * class 1
	 */
	public static void initialize() { 
		Random r = new Random(System.currentTimeMillis());
		
		// Ensure that some directories exist
		File crossDir = new File("data/cross-validation/");
		if (!crossDir.exists())
			crossDir.mkdir();
		
		File kDir = new File("data/cross-validation/k" + FOLDS + "/");
		if (!kDir.exists())
			kDir.mkdir();

		// two classes - niall-f and niall-g
		List<String> classNames = new ArrayList<String>();
		classNames.add("niall-f");
		classNames.add("niall-g");
		List<Map<String,List<Integer>>> sets = new ArrayList<Map<String,List<Integer>>>();
		for (int i = 0; i < FOLDS; i++) {
			Map<String,List<Integer>> map = new TreeMap<String,List<Integer>>();
			for (String c : classNames) 
				map.put(c, new ArrayList<Integer>());
			sets.add(map);
		}

		// Randomly select some of the instances
		// for each of the folds.
		for (String className : classNames) { 
			List<Integer> episodes = new ArrayList<Integer>();
			for (int i = 1; i <= N; ++i) 
				episodes.add(i);
			Collections.shuffle(episodes, r);
			
			for (int i = 0; i < episodes.size(); ++i) {
				sets.get(i % FOLDS).get(className).add(episodes.get(i));
			}
		}

		Experiments.writeTestFile("niall", classNames, sets, FOLDS);
		
		// create the folder for signatures trained from sequences of
		// Allen relations.
		for (int i = 0; i < FOLDS; i++) {
			String f = "data/cross-validation/k" + FOLDS + "/fold-" + i + "/allen/";
				File file = new File(f);
				if (!file.exists())
					file.mkdir();
		}		
	}
	
	/**
	 * Class 1 does not change -- so we need to construct the niall-f.lisp file
	 */
	public static void generateClass1() { 
		try {
			// initial arguments
			//   prefix - where you want the file written
			//   p - the number of streams.
			//   alphabet.size - the number of symbols (constant in each stream)
			//   episode.length -- the length of each episode
			String prefix = "/tmp/niall/";
			int streams = 6;
			int size = 8;
			int eLength = 200;
			
			String cmd = "scripts/sim.R 1 " + prefix + " " + streams + " " + size + " " + eLength;
			System.out.println(cmd);
			Process p = Runtime.getRuntime().exec("Rscript " + cmd);
			p.waitFor();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		SymbolicData.convert("/tmp/niall/f", "data/input/niall-f.lisp", N);
		Experiments.signatures("niall", SequenceType.allen, FOLDS, true);
	}
	
	public static void experiment1() { 
		int[] episodeLengths = new int[] { 200, 400, 600, 800, 1000 };
		double[] means = new double[] { 0.00, 0.5, 1.0, 1.5, 2.0 };
		
		// Total number of experiments equals 25
		for (double mean : means) { 
			for (int episodeLength : episodeLengths) { 
				
			}
		}
	}
}
