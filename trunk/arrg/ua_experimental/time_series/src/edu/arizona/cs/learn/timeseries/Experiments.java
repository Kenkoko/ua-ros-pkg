package edu.arizona.cs.learn.timeseries;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.classification.ClassifyCallable;
import edu.arizona.cs.learn.timeseries.classification.ClassifyResults;
import edu.arizona.cs.learn.timeseries.evaluation.FoldStatistics;
import edu.arizona.cs.learn.timeseries.model.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizeCallable;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizeResults;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizerStatistics;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class Experiments {
	private static Logger logger = Logger.getLogger(Experiments.class);
	private String _sequenceType;
	private String _classify;

	private boolean _shuffle;
	
	private int _kFolds;
	private int _k;
	private int _cavePct;
	
	private boolean _fromFile;
	private ExecutorService _execute;

	public Experiments(int kFolds) {
		this._kFolds = kFolds;
	}

	public Experiments(String sequence, String classify, int kFolds,
			boolean shuffle, boolean fromFile) {
		this(kFolds);

		this._shuffle = shuffle;
		this._sequenceType = sequence;
		this._classify = classify;
		this._k = 10;
		this._cavePct = 50;

		this._fromFile = fromFile;
	}

	public void setCAVEPercent(int value) {
		this._cavePct = value;
	}

	public void setK(int value) {
		this._k = value;
	}

	public int getFolds() {
		return this._kFolds;
	}

	private void writeMatrix(String c, String g, List<String> classNames,
			int[][] matrix) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/" + c
					+ "-" + g + "-matrix.csv"));
			for (String s : classNames) {
				out.write("," + s);
			}
			out.write("\n");

			for (int i = 0; i < classNames.size(); i++) {
				out.write((String) classNames.get(i));
				for (int j = 0; j < classNames.size(); j++) {
					out.write("," + matrix[i][j]);
				}
				out.write("\n");
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void writeLog(String activityName, String classifierName,
			List<SequenceType> types, List<List<FoldStatistics>> values) {
		try {
			File f = new File("logs/" + activityName + "-" + classifierName
					+ ".csv");
			BufferedWriter out = new BufferedWriter(new FileWriter(f));

			StringBuffer buf = new StringBuffer();
			buf.append("activity,classifier,cavePct,sequence,fold,accuracy,avgTrainingSequenceLength,avgTestingSequenceLength,trainingTime,testingTime,avgSignatureSize");

			out.write(buf.toString() + "\n");
			System.out.println(buf.toString());

			for (int i = 0; i < types.size(); i++) {
				SequenceType type = types.get(i);
				List<FoldStatistics> results = values.get(i);
				for (int j = 0; j < results.size(); j++) {
					FoldStatistics stats = (FoldStatistics) results.get(j);
					buf = new StringBuffer();
					buf.append(activityName + "," + classifierName + ","
							+ this._cavePct + "," + type + "," + j + ","
							+ stats.accuracy + ",");
					buf.append(stats.trainingLengths.getMean() + ","
							+ stats.testingLengths.getMean() + ",");
					buf.append(stats.trainingTime + "," + stats.testingTime
							+ ",");
					buf.append(stats.signatureSize.getMean());

					out.write(buf.toString() + "\n");
					System.out.println("," + buf.toString());
				}
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void appendANOVA(String c, String line) {
		try {
			File f = new File("data/anova/" + c + "-ANOVA.csv");
			boolean writeHeader = true;
			if (f.exists()) {
				writeHeader = false;
			}
			BufferedWriter out = new BufferedWriter(new FileWriter(f, true));

			if (writeHeader) {
				out.write("activity,representation,fold,correct\n");
			}
			out.write(line + "\n");
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public List<FoldStatistics> crossValidationLite(String activityName, Classifier c, 
			SequenceType type, List<String> classNames) {
		
		logger.debug("Beginning cross validation [lite]");
		_execute = Executors.newFixedThreadPool(Utils.numThreads);
		
		Map<String,List<Episode>> data = Utils.loadAllEpisodes(activityName);

		List<FoldStatistics> foldStats = new ArrayList<FoldStatistics>();
		int[][] matrix = new int[classNames.size()][classNames.size()];

		for (int i = 0; i < this._kFolds; i++) {
			logger.debug("Fold --- " + i);
			FoldStatistics fs = new FoldStatistics();

			Map<String,List<Integer>> testMap = Utils.getTestSet(activityName, _kFolds, i);
			
			List<Episode> training = new ArrayList<Episode>();
			List<Instance> test = new ArrayList<Instance>();
			for (String key : data.keySet()) {
				List<Episode> list = data.get(key);
				List<Integer> ignore = testMap.get(key);

				for (Episode episode : list) {
					if (!ignore.contains(episode.id())) { 
						training.add(episode);
					} else { 
						Instance instance = episode.toInstance(type);
						if (Utils.testExcludeSet.size() > 0) { 
							test.add(instance.copy());
						} else { 
							test.add(instance);
						}
					}
				}

			}

			long trainStart = System.currentTimeMillis();
			c.trainEpisodes(i, training, type, _shuffle);
			long trainEnd = System.currentTimeMillis();
			fs.trainingTime = (trainEnd - trainStart);

			c.addData(fs);

			double right = 0.0D;
			int wrong = 0;

			List<Future<ClassifyResults>> future = new ArrayList<Future<ClassifyResults>>();

			for (Instance instance : test) {
				fs.testingLengths.addValue(instance.sequence().size());
				future.add(_execute.submit(new ClassifyCallable(c, instance)));
			}

			for (Future<ClassifyResults> results : future) {
				ClassifyResults thread = null;
				try {
					thread = results.get();
				} catch (InterruptedException e) {
					e.printStackTrace();
				} catch (ExecutionException e) {
					e.printStackTrace();
				}

				String className = thread.className;
				Instance instance = thread.test;
				fs.testingTime += thread.elapsed;

				int correctIndex = classNames.indexOf(instance.name());
				int classifyIndex = classNames.indexOf(className);
				matrix[correctIndex][classifyIndex] += 1;

				StringBuffer buf = new StringBuffer();
				buf.append(activityName + "," + type.toString() + "," + i + ",");

				int result = 0;
				if (className.equals(instance.name())) {
					result = 1;
					right += 1.0D;
				} else {
					wrong++;
				}

				buf.append(result);
				appendANOVA(c.getName(), buf.toString());
			}

			fs.accuracy = (right / test.size());
			foldStats.add(fs);
		}

		writeMatrix(c.getName(), type.toString(), classNames, matrix);

		this._execute.shutdown();
		return foldStats;
	}	
	
	public List<FoldStatistics> crossValidation(String activityName, Classifier c, 
			SequenceType type, List<String> classNames) {
		logger.debug("Beginning cross validation");

		_execute = Executors.newFixedThreadPool(Utils.numThreads);

		Map<String,List<Instance>> data = Utils.load(activityName, type);
		if (_shuffle) {
			for (List<Instance> list : data.values()) {
				for (Instance instance : list) {
					instance.shuffle();
				}
			}

		}

		List<FoldStatistics> foldStats = new ArrayList<FoldStatistics>();
		int[][] matrix = new int[classNames.size()][classNames.size()];

		for (int i = 0; i < _kFolds; i++) {
			logger.debug("Fold --- " + i);
			FoldStatistics fs = new FoldStatistics();

			Map<String,List<Integer>> testMap = Utils.getTestSet(activityName, _kFolds, i);

			List<Instance> training = new ArrayList<Instance>();
			List<Instance> test = new ArrayList<Instance>();
			for (String key : data.keySet()) {
				List<Instance> list = data.get(key);
				List<Integer> ignore = testMap.get(key);

				for (Instance instance : list) {
					if (!ignore.contains(instance.id())) { 
						training.add(instance);
						fs.trainingLengths.addValue(instance.sequence().size());
					} else if (Utils.testExcludeSet.size() > 0) {
						test.add(instance.copy());
					} else {
						test.add(instance);
					}

				}

			}

			long trainStart = System.currentTimeMillis();
			c.train(i, training);
			long trainEnd = System.currentTimeMillis();
			fs.trainingTime = (trainEnd - trainStart);

			c.addData(fs);

			double right = 0.0D;
			int wrong = 0;

			List<Future<ClassifyResults>> future = new ArrayList<Future<ClassifyResults>>();

			for (Instance instance : test) {
				fs.testingLengths.addValue(instance.sequence().size());
				future.add(_execute.submit(new ClassifyCallable(c, instance)));
			}

			for (Future<ClassifyResults> results : future) {
				ClassifyResults thread = null;
				try {
					thread = results.get();
				} catch (InterruptedException e) {
					e.printStackTrace();
				} catch (ExecutionException e) {
					e.printStackTrace();
				}

				String className = thread.className;
				Instance instance = thread.test;
				fs.testingTime += thread.elapsed;

				int correctIndex = classNames.indexOf(instance.name());
				int classifyIndex = classNames.indexOf(className);
				matrix[correctIndex][classifyIndex] += 1;

				StringBuffer buf = new StringBuffer();
				buf.append(activityName + "," + type.toString() + "," + i + ",");

				int result = 0;
				if (className.equals(instance.name())) {
					result = 1;
					right += 1.0D;
				} else {
					wrong++;
				}

				buf.append(result);
				appendANOVA(c.getName(), buf.toString());
			}

			fs.accuracy = (right / test.size());
			foldStats.add(fs);
		}

		writeMatrix(c.getName(), type.toString(), classNames, matrix);

		_execute.shutdown();
		return foldStats;
	}

	public void classifySplit(String activityName, Classifier c,
			SequenceType type, List<String> classNames) {
		this._execute = Executors.newFixedThreadPool(Utils.numThreads);

		Map<String,List<Instance>> data = Utils.load(activityName, type);
		if (_shuffle) {
			for (List<Instance> list : data.values()) {
				for (Instance instance : list) {
					instance.shuffle();
				}
			}
		}

		List<Instance> training = new ArrayList<Instance>();
		List<Instance> test = new ArrayList<Instance>();
		for (String activity : data.keySet()) {
			List<Instance> episodes = data.get(activity);
			Collections.shuffle(episodes);

			int twoThirds = (int) Math.floor(0.6666 * episodes.size());
			for (int i = 0; i < twoThirds; i++) {
				training.add((Instance) episodes.get(i));
			}

			for (int i = twoThirds; i < episodes.size(); i++) {
				test.add(episodes.get(i));
			}
		}

		c.train(training);

		List<Future<ClassifyResults>> future = new ArrayList<Future<ClassifyResults>>();
		for (Instance instance : test) {
			future.add(_execute.submit(new ClassifyCallable(c, instance)));
		}

		for (Future<ClassifyResults> results : future) {
			ClassifyResults thread = null;
			try {
				thread = results.get();
			} catch (InterruptedException e) {
				e.printStackTrace();
			} catch (ExecutionException e) {
				e.printStackTrace();
			}

			String className = thread.className;
			Instance instance = thread.test;

			int result = 0;
			if (className.equals(instance.name())) {
				result = 1;
			}
			StringBuffer buf = new StringBuffer();
			buf.append(activityName + "," + type.toString() + ",," + result);

			appendANOVA(c.getName(), buf.toString());
		}

		this._execute.shutdown();
	}

	public Map<String, RecognizerStatistics> recognition(String dataset, 
			Recognizer r, SequenceType type, 
			int minPct, boolean prune, boolean onlyStart) {

		_execute = Executors.newFixedThreadPool(Utils.numThreads);

		List<String> classes = Utils.getActivityNames(dataset);
		Map<String,Map<Integer,List<Interval>>> data = new HashMap<String,Map<Integer,List<Interval>>>();
		for (String className : classes) {
			data.put(className, Utils.load(new File("data/input/" + className + ".lisp")));
		}

		Map<String,RecognizerStatistics> map = new HashMap<String,RecognizerStatistics>();
		for (String className : classes)
			map.put(className, new RecognizerStatistics(className));

		for (int i = 0; i < _kFolds; i++) {
			Map<String, List<Integer>> testMap = Utils.getTestSet(dataset, _kFolds, i);
			List<FSMRecognizer> recognizers = new ArrayList<FSMRecognizer>();

			double testSize = 0.0D;

			String dir = "data/cross-validation/k" + _kFolds + "/fold-" + i + "/" + type + "/";
			String suffix = ".xml";
			if (prune)
				suffix = "-prune.xml";

			for (String className : data.keySet()) {
				String signatureFile = dir + className + suffix;
				FSMRecognizer mr = r.build(className, signatureFile, data.get(className), 
						testMap.get(className), minPct, onlyStart);
				recognizers.add(mr);
			}

			List<Future<RecognizeResults>> future = new ArrayList<Future<RecognizeResults>>();
			for (String className : testMap.keySet()) {
				for (Integer id : testMap.get(className)) {
					List<Interval> testItem = data.get(className).get(id);
					RecognizeCallable rc = new RecognizeCallable(recognizers, className, id, testItem);
					future.add(_execute.submit(rc));
				}

				testSize += testMap.get(className).size();
			}

			for (Future<RecognizeResults> results : future) {
				RecognizeResults rr = null;
				try {
					rr = results.get();
				} catch (Exception e) {
					e.printStackTrace();
				}

				for (String className : classes) {
					RecognizerStatistics rs = map.get(className);

					if (className.equals(rr.testClass)) {
						if (rr.recognized.get(className)) { 
							rs.truePositives.add(Integer.valueOf(rr.testId));
						} else {
							rs.falseNegative.add(Integer.valueOf(rr.testId));
						}
					} else if (rr.recognized.get(className)) { 
						rs.falsePositive(rr.testClass, rr.testId);
					} else { 
						rs.trueNegative(rr.testClass, rr.testId);
					}
				}
			}
		}

		for (String className : map.keySet()) { 
			RecognizerStatistics rs = map.get(className);
			logger.debug("--- " + className);
			logger.debug("        -- " + rs.tp() + "\t" + rs.fp());
			logger.debug("        -- " + rs.fn() + "\t" + rs.tn());
			logger.debug("      precision: " + rs.precision());
			logger.debug("      recall: " + rs.recall());
			logger.debug("      f-measure: " + rs.fscore());
		}

		_execute.shutdown();
		return map;
	}

	public void runComparison(String prefix, boolean lite) {
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;
		AllenRelation.MARGIN = 5.0F;

		List<String> fileNames = Utils.getActivityNames(prefix);
		logger.debug("Comparison for : " + prefix);

		List<Classify> classifyList = Classify.get(_classify);
		for (Classify c : classifyList) {
			List<SequenceType> typesTested = new ArrayList<SequenceType>();
			List<List<FoldStatistics>> testResults = new ArrayList<List<FoldStatistics>>();

			StringBuffer buf = new StringBuffer();

			buf.append(prefix + " & ");
			for (SequenceType type : SequenceType.get(_sequenceType)) {
				buf.append(type + " & ");
			}
			buf.append("\n");

			buf.append(prefix + " & ");
			for (SequenceType type : SequenceType.get(this._sequenceType)) {
				Classifier classifier = c.getClassifier(type, _k, _cavePct, _fromFile, getFolds());
				List<FoldStatistics> values = new ArrayList<FoldStatistics>();
				if (lite)
					values = crossValidationLite(prefix, classifier, type, fileNames);
				else
					values = crossValidation(prefix, classifier, type, fileNames);
				SummaryStatistics ss = new SummaryStatistics();
				for (FoldStatistics fd : values) {
					ss.addValue(fd.accuracy);
				}
				buf.append(toString(ss) + " & ");

				typesTested.add(type);
				testResults.add(values);
			}
			buf.append(" \\\\ \\hline");
			writeLog(prefix, c.toString(), typesTested, testResults);
			System.out.println(buf.toString());
		}
	}

	public static void main(String[] args) {
	}

	private static String toString(SummaryStatistics ss) {
		double meanPct = 100.0D * ss.getMean();
		double meanStd = 100.0D * ss.getStandardDeviation();

		return Utils.nf.format(meanPct) + "\\% & " + Utils.nf.format(meanStd);
	}

	public static void init(String pre, String c, String s, int pct, int k,
			int folds, boolean shuffle, boolean load, boolean light) {
		Experiments cv = new Experiments(s, c, folds, shuffle, load);
		cv.setCAVEPercent(pct);
		cv.setK(k);

		List<String> prefixes = Utils.getPrefixes(pre);
		for (String prefix : prefixes)
			cv.runComparison(prefix, light);
	}

	public static void selectCrossValidation(String prefix, int folds) {
		Random r = new Random(System.currentTimeMillis());
		
		// Ensure that some directories exist
		File crossDir = new File("data/cross-validation/");
		if (!crossDir.exists())
			crossDir.mkdir();
		
		File kDir = new File("data/cross-validation/k" + folds + "/");
		if (!kDir.exists())
			kDir.mkdir();

		List<String> classNames = new ArrayList<String>();
		for (File f : new File("data/input/").listFiles()) {
			if ((f.getName().startsWith(prefix))
					&& (f.getName().endsWith("lisp"))) {
				String name = f.getName();
				classNames.add(name.substring(0, name.indexOf(".lisp")));
			}

		}

		int k = folds;
		List<Map<String,List<Integer>>> sets = new ArrayList<Map<String,List<Integer>>>();
		for (int i = 0; i < k; i++) {
			Map<String,List<Integer>> map = new TreeMap<String,List<Integer>>();
			for (String c : classNames) 
				map.put(c, new ArrayList<Integer>());
			sets.add(map);
		}

		for (String className : classNames) { 
			String f = "data/input/" + className + ".lisp";
			Map<Integer,List<Interval>> map = Utils.load(new File(f));
			List<Integer> episodes = new ArrayList<Integer>(map.keySet());
			Collections.shuffle(episodes, r);
			
			for (int i = 0; i < episodes.size(); ++i) {
				sets.get(i % k).get(className).add(episodes.get(i));
			}
		}

		try {
			for (int i = 0; i < k; i++) {
				String dir = "data/cross-validation/k" + k + "/fold-" + i + "/";
				if (!new File(dir).exists()) {
					new File(dir).mkdir();
				}

				String f = dir + prefix + "-test.txt";
				BufferedWriter out = new BufferedWriter(new FileWriter(f));
				for (String c : classNames) {
					out.write(c);
					
					for (Integer id : sets.get(i).get(c)) { 
						out.write(" " + id);
					}
					out.write("\n");
				}
				out.close();
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

		List<SequenceType> types = SequenceType.get("all");
		types.add(SequenceType.bpp);
		for (SequenceType type : SequenceType.get("all"))
			for (int i = 0; i < k; i++) {
				String f = "data/cross-validation/k" + k + "/fold-" + i + "/"
						+ type + "/";
				File file = new File(f);
				if (!file.exists())
					file.mkdir();
			}
	}

	public static void signatures(final String prefix, final SequenceType type, final int folds, final boolean prune) {
		// We should be able to multithread across each of the folds since they are independent
		// of each other.  The only challenge will be get the test set, but I think that it
		// should work.
		class SignatureCallable implements Callable<Object> { 
			private int _fold;
			
			public SignatureCallable(int fold) { 
				_fold = fold;
			}

			@Override
			public Object call() throws Exception {
				System.out.println("Fold " + (_fold+1));
				List<String> activityNames = Utils.getActivityNames(prefix);
				Map<String,List<Integer>> testMap = Utils.getTestSet(prefix, folds, _fold);
				
				for (String activity : activityNames) { 
					System.out.println("..Building signature for " + activity);
					File dataFile = new File("data/input/" + activity + ".lisp");
					List<Instance> instances = Utils.sequences(activity, dataFile.getAbsolutePath(), type);
					
					String f = "data/cross-validation/k" + folds + "/fold-" + _fold + "/" + type + "/";
					File file = new File(f);
					if (!file.exists()) {
						file.mkdir();
					}

					Signature s = new Signature(activity);

					List<Integer> testSet = testMap.get(activity);
					for (int i = 1; i <= instances.size(); i++) {
						Instance instance = instances.get(i - 1);
						if (testSet.contains(instance.id()))  
							continue;

						s.update(instance.sequence());
						if (prune && (i % 10 == 0)) 
							s = s.prune(3);
					}

					// remove all of the excess and make sure that what
					// we write out could be as small as possible.
					int half = s.trainingSize() / 2;
					s = s.prune(half);

					if (prune)
						s.toXML(f + activity + "-prune.xml");
					else
						s.toXML(f + activity + ".xml");

				}
				return new Object();
			}
			
		}
		
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		for (int  fold = 0; fold < folds; ++fold) { 			
			execute.submit(new SignatureCallable(fold));
		}
		
		try {
			execute.awaitTermination(20, TimeUnit.DAYS);
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}

	public static void signatureTiming(String prefix, SequenceType type, int folds, boolean prune) {
		try {
			String log = "logs/" + prefix + "-" + type + "-" + folds + "-"
					+ prune + "-timing.csv";
			BufferedWriter out = new BufferedWriter(new FileWriter(log));

			out.write("prefix,activity,type,folds,prune,fold,time\n");
			for (int fold = 0; fold < folds; fold++) {
				System.out.println("Fold -- " + fold);
				Map<String,List<Instance>> map = Utils.load(prefix, type);
				Map<String,List<Integer>> testMap = Utils.getTestSet(prefix, folds, fold);

				for (String key : map.keySet()) {
					long start = System.currentTimeMillis();
					Signature s = new Signature(key);

					List<Integer> testSet = testMap.get(key);
					List<Instance> list = map.get(key);
					for (int i = 1; i <= list.size(); i++) {
						Instance instance = (Instance) list.get(i - 1);
						if (testSet.indexOf(Integer.valueOf(instance.id())) != -1) {
							continue;
						}
						s.update(((Instance) list.get(i - 1)).sequence());
						if ((prune) && (i % 10 == 0)) {
							s = s.prune(3);
						}
					}
					long end = System.currentTimeMillis();
					long elapsed = end - start;

					out.write(prefix + "," + key + "," + type + "," + folds
							+ "," + prune + "," + fold + "," + elapsed + "\n");
				}
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}