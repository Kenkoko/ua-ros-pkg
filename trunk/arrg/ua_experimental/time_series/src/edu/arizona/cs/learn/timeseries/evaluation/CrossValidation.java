package edu.arizona.cs.learn.timeseries.evaluation;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.evaluation.classifier.Classifier;
import edu.arizona.cs.learn.timeseries.model.AllenRelation;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class CrossValidation {
    private static Logger logger = Logger.getLogger(CrossValidation.class);
    
    private String _sequenceType;
    private String _classify;
    
    private int _cavePct;
    
    private ExecutorService _execute;

    public CrossValidation(String sequence, String classify) { 
    	_sequenceType = sequence; 
    	_classify = classify;
    	_cavePct = 50;
	}
    
    public void setCAVEPercent(int value) { 
    	_cavePct = value;
    }
	
	/**
	 * 
	 * @param c
	 * @param classNames
	 * @param matrix
	 */
	private void writeMatrix(String c, String g, List<String> classNames, int[][] matrix) { 
		// write out the matrix
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/" + c + "-" + g + "-matrix.csv"));
			for (String s : classNames) { 
				out.write("," + s);
			}
			out.write("\n");
			
			for (int i = 0; i < classNames.size(); ++i) { 
				out.write(classNames.get(i));
				for (int j = 0; j < classNames.size(); ++j) { 
					out.write("," + matrix[i][j]);
				}
				out.write("\n");
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}		
	}
	
	/**
	 * Record all of the details of what happened to the file given.
	 * @param classifierName
	 * @param activityName
	 * @param representation
	 * @param values
	 */
	private void writeLog(String activityName, String classifierName, List<SequenceType> types, List<List<FoldStatistics>> values) { 
		try { 
			File f = new File("logs/" + activityName + "-" + classifierName + ".csv");
			BufferedWriter out = new BufferedWriter(new FileWriter(f));
			
			out.write("activity,classifier,cavePct,sequence,fold,accuracy,avgTrainingSequenceLength,avgTestingSequenceLength,trainingTime,testingTime,avgSignatureSize\n");
			for (int i = 0; i < types.size(); ++i) { 
				SequenceType type = types.get(i);
				List<FoldStatistics> results = values.get(i);
				for (int j = 0; j < results.size(); ++j) { 
					FoldStatistics stats = results.get(j);
					out.write(activityName + "," + classifierName + "," + _cavePct + "," + type + "," + j + "," + stats.accuracy + ",");
					out.write(stats.trainingLengths.getMean() + "," + stats.testingLengths.getMean() + ",");
					out.write(stats.trainingTime + "," + stats.testingTime + ",");
					out.write(stats.signatureSize.getMean() + "");
					out.write("\n");
				}
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}		

	}
	
	/**
	 * 
	 * @param c
	 * @param activityName
	 * @param representation
	 * @param values
	 */
	private void appendANOVA(String c, String activityName, String representation, List<FoldStatistics> values) { 
		// write out the matrix
		try { 
			File f = new File("/tmp/" + c + "-ANOVA.csv");
			boolean writeHeader = true;
			if (f.exists())
				writeHeader = false;
			
			BufferedWriter out = new BufferedWriter(new FileWriter(f, true));
			
			if (writeHeader)
				out.write("activity,representation,value\n");
			
			for (FoldStatistics fs : values)  
				out.write(activityName+ "," + representation + "," + fs.accuracy + "\n");
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}		
	}	
	
	public List<FoldStatistics> doit(String activityName, Classifier c, 
			SequenceType type, List<String> classNames, 
			int k, boolean reverseSequences) {
    	_execute = Executors.newFixedThreadPool(Utils.numThreads);
		Random r = new Random(System.currentTimeMillis());

		File saveDir = new File("/tmp/printing/");
		if (saveDir.exists()) { 
			for (File f : saveDir.listFiles())
				f.delete();
		} else { 
			saveDir.mkdir();
		}
		
		String prefix = "/tmp/" + c.getName() + "-errors/";
		File dir = new File(prefix);
		if (!dir.exists()) 
			dir.mkdir();

		// partition the training sets into k different subsets.
		List<List<Instance>> sets = new ArrayList<List<Instance>>();
		for (int i = 0; i < k; ++i) { 
			sets.add(new ArrayList<Instance>());
		}
		
		for (String className : classNames) {
			String f = "data/input/" + className + ".lisp";
			
			// Load the sequences, shuffle them and add them to a set.
			List<Instance> list = Utils.sequences(className, f, type);
			if (reverseSequences) { 
				for (Instance i : list) { 
					i.reverse();
				}
			}
			Collections.shuffle(list, r);
			for (int i = 0; i < list.size(); ++i) { 
				sets.get(i%k).add(list.get(i));
			}
		}
		
		List<FoldStatistics> foldStats = new ArrayList<FoldStatistics>();
		int[][] matrix = new int[classNames.size()][classNames.size()];

		// for each fold
		for (int i = 0; i < k; ++i) { 
			FoldStatistics fs = new FoldStatistics();
			
			// determine the average sequence length of the
			// training set
			for (int j = 0; j < sets.size(); ++j) { 
				if (i == j) 
					continue;
				for (Instance instance : sets.get(i)) { 
					fs.trainingLengths.addValue(instance.sequence().size());
				}
			}			
			
			// do the training careful to record the start and 
			// end time.
			long trainStart = System.currentTimeMillis();
			c.train(sets, i);
			long trainEnd = System.currentTimeMillis();
			fs.trainingTime = (trainEnd - trainStart) ;

			c.addData(fs);
			
			double right = 0;
			int wrong = 0;
			List<Instance> testSet = sets.get(i);
			
			List<Future<TestResults>> future = new ArrayList<Future<TestResults>>();

//			logger.debug("Fold: " + i + " Training complete.  Number of tests: " + testSet.size());
			for (Instance instance : testSet) { 
				fs.testingLengths.addValue(instance.sequence().size());
				
				// We have to make a copy of the instance so that we can remove
				// all of the unobservables.
				future.add(_execute.submit(new TestCallable(c, instance)));
			}
			
			for (Future<TestResults> results : future) { 
				TestResults thread = null;
				try {
					thread = results.get();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ExecutionException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				String className = thread.className;
				Instance instance = thread.test;
				fs.testingTime += thread.elapsed;

				int correctIndex = classNames.indexOf(instance.name());
				int classifyIndex = classNames.indexOf(className);
				matrix[correctIndex][classifyIndex] += 1;
				
				if (className.equals(instance.name())) { 
					right += 1;
				} else { 
					++wrong;
//					instance.write(prefix + i + "-" + wrong + "-" + instance.getKey() + "-as-" + className + ".sequence");
//					c.printError(tmpInstance, (int) total);
				}
			}
			
			fs.accuracy = right / (double) testSet.size();
			foldStats.add(fs);
		}

		writeMatrix(c.getName(), type.name(), classNames, matrix);
		appendANOVA(c.getName(), activityName, type.name(), foldStats);
		
		_execute.shutdown();
		return foldStats;
	}
	
	public void runPowerComparison(String prefix) { 
		if (prefix.equals("wes") || prefix.equals("nicole") || prefix.equals("derek")) { 
			logger.debug("Removing SAX");
			Utils.excludeSet = Exclude.handwritingExcludeSAX;
			runComparison(prefix, false);

			logger.debug("Removing SDL");
			Utils.excludeSet = Exclude.handwritingExcludeSDL;
			runComparison(prefix, false);
		} else { 
			logger.debug("Removing SAX");
			Utils.excludeSet = Exclude.excludeSAX;
			runComparison(prefix, false);

			logger.debug("Removing SDL");
			Utils.excludeSet = Exclude.excludeSDL;
			runComparison(prefix, false);
		}
	}

		
	public void runComparison(String prefix, boolean reverseSequences) { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;
		AllenRelation.MARGIN = 5;
		
    	List<String> fileNames = new ArrayList<String>();
    	for (File f : new File("data/input/").listFiles()) {
    		if (f.getName().startsWith(prefix) && f.getName().endsWith("lisp")) { 
    			String name = f.getName();
    			fileNames.add(name.substring(0, name.indexOf(".lisp")));
    		}
    	}
    	logger.debug("Comparison for : " + prefix);
    	
    	List<Classify> classifyList = Classify.get(_classify);
    	for (Classify c : classifyList) { 
    		runSingleComparison(prefix, c.getClassifier(10, _cavePct), fileNames, reverseSequences);
    	}
	}
	
	public void runSingleComparison(String name, Classifier c, List<String> fileNames, boolean reverseSequences) { 
    	logger.debug("running ... " + c.getName());
    	
    	List<SequenceType> typesTested = new ArrayList<SequenceType>();
    	List<List<FoldStatistics>> testResults = new ArrayList<List<FoldStatistics>>();

		StringBuffer buf = new StringBuffer();
		
		buf.append(name + " & ");
		for (SequenceType type : SequenceType.get(_sequenceType)) { 
			buf.append(type + " & ");
		}
		buf.append("\n");
		
		buf.append(name + " & ");
		for (SequenceType type : SequenceType.get(_sequenceType)) { 
			List<FoldStatistics> values = doit(name, c, type, fileNames, 10, reverseSequences);
			SummaryStatistics ss = new SummaryStatistics();
			for (FoldStatistics fd : values)
				ss.addValue(fd.accuracy);
			
			buf.append(toString(ss) + " & ");
			
			typesTested.add(type);
			testResults.add(values);
		}
    	buf.append(" \\\\ \\hline");
		writeLog(name, c.getName(), typesTested, testResults);		
    	System.out.println(buf.toString());
	}
	
	/**
	 * Used in order to create Latex strings.  Just formats and outputs the
	 * mean and standard deviation of the summary statistics.
	 * @param ss
	 * @return
	 */
	private static String toString(SummaryStatistics ss) { 
		double meanPct = (100.0 * ss.getMean());
		double meanStd = (100.0 * ss.getStandardDeviation());
		
		return Utils.nf.format(meanPct) + "\\% & " + Utils.nf.format(meanStd);
	}
	
	public static void init(String task, String pre, String c, String s, int pct) {
		CrossValidation cv = new CrossValidation(s, c);
		cv.setCAVEPercent(pct);
		
		List<String> prefixes = Utils.getPrefixes(pre);
		

		if (task.equals("comparison") || task.equals("comp")) { 
			for (String prefix : prefixes) 
				cv.runComparison(prefix, false);
		} else if (task.equals("power")) { 
			for (String prefix : prefixes)  
				cv.runPowerComparison(prefix);
		} else { 
			printError();
			return;
		}
		
	}
	
	/**
	 * The main method.  Critical for running. Ha!
	 * @param args
	 */
	public static void main(String[] args) { 
		if (args.length != 6) { 
			printError();
			return;
		}
		
		Utils.numThreads = Integer.parseInt(args[5]);
		init(args[0], args[1], args[2], args[3], Integer.parseInt(args[4]));
}	
	
	private static void printError() { 
		logger.error("Usage: CrossValidation experiment files classifcation sequenceType cavePct numThreads");
		logger.error("experiment - comparison/power");
		logger.error("files - prefix/easy/hard/all");
		
		StringBuffer buf = new StringBuffer();
		for (Classify c : Classify.values()) { 
			buf.append(c.toString() + "/");
		}
		buf.deleteCharAt(buf.length()-1);
		logger.error("Classification - " + buf);
		
		buf = new StringBuffer();
		for (SequenceType type : SequenceType.values()) { 
			buf.append(type.toString() + "/");
		}
		buf.deleteCharAt(buf.length()-1);
		logger.error("SequenceType - " + buf);
		logger.error("cavePct 0-100");
		logger.error("numThreads - 1,2,3,4...");
	}
	
	/**
	 * This method will determine the power of SAX and SDL 
	 * on the handwriting data since it is slightly different 
	 * than the other real-valued to proposition data
	 */
//	public static void handwritingPower() { 
//		logger.debug("Removing SAX");
//		SequenceFactory.excludeSet = Exclude.handwritingExcludeSAX;
//		runComparison("wes", false);
//		runComparison("nicole", false);
//		runComparison("derek", false);
//		
//		logger.debug("Removing SDL");
//		SequenceFactory.excludeSet = Exclude.handwritingExcludeSDL;
//		runComparison("wes", false);
//		runComparison("nicole", false);
//		runComparison("derek", false);
//		
//	}
//	
//	public static void power() { 
//		logger.debug("Removing SAX");
//		SequenceFactory.excludeSet = Exclude.excludeSAX;
//		runComparison("wafer", false);
//		runComparison("ecg", false);
//
//		logger.debug("Removing SDL");
//		SequenceFactory.excludeSet = Exclude.excludeSDL;
//		runComparison("wafer", false);
//		runComparison("ecg", false);
//
//		logger.debug("Removing SAX");
//		SequenceFactory.excludeSet = Exclude.excludeSAX;
//		runComparison("ww2d", false);
//		runComparison("tctodd", false);
//		
//		logger.debug("Removing SDL");
//		SequenceFactory.excludeSet = Exclude.excludeSDL;
//		runComparison("ww2d", false);
//		runComparison("tctodd", false);
//	}
	
	
    class TestResults { 
    	public String className;
    	public Classifier classifier;
    	public Instance test;
    	public double elapsed;
    	
    	public TestResults(Classifier c, Instance test, String className, double elapsed) { 
    		this.className = className;
    		this.classifier = c;
    		this.test = test;
    		this.elapsed = elapsed;
    	}
    }
    
	class TestCallable implements Callable<TestResults> { 
		public String className;
		
		public Classifier classifier;
		public Instance test;
		
		public TestCallable(Classifier c, Instance test) { 
			this.classifier = c;
			this.test = test;
		}
		
		@Override
		public TestResults call() throws Exception {
			long testStart = System.currentTimeMillis();
			className = classifier.test(test);
			long testEnd = System.currentTimeMillis();
			return new TestResults(classifier, test, className, testEnd - testStart);
		}
	}
}
