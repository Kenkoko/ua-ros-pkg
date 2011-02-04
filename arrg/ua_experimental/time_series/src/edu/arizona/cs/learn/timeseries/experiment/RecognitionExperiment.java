package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.evaluation.BatchSignatures;
import edu.arizona.cs.learn.timeseries.evaluation.CrossValidation;
import edu.arizona.cs.learn.timeseries.evaluation.SplitAndTest;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizeCallable;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizerStatistics;
import edu.arizona.cs.learn.util.Utils;

public class RecognitionExperiment {
	private static Logger logger = Logger.getLogger(RecognitionExperiment.class);
	private static String inputDir = "/Users/wkerr/Sync/data/handwriting/wes/lisp/original/";

	private String _prefix;
	private int _prunePct;
	
	private BufferedWriter _pOut;
	private BufferedWriter _sOut;
	
	public RecognitionExperiment(String prefix, int prunePct, boolean writeSample) throws Exception { 
		_prefix = prefix;
		_prunePct = prunePct;
		
		_pOut = new BufferedWriter(new FileWriter(inputDir + _prefix + "-recognition-performance.csv"));
		_pOut.write("batch_id,split_pct,id,name,r_name,accepted,tp,fp,tn,fn\n");
		_pOut.flush();
		
		if (writeSample) {
			_sOut = new BufferedWriter(new FileWriter(inputDir + _prefix + "-recognition-samples.csv"));
			_sOut.write("batch_id,split_pct,id,name,r_name,time_step,index,depth,ratio\n");
		}
	}
	
	public void done() throws Exception { 
		_pOut.close();
		
		if (_sOut != null)
			_sOut.close();
	}
	
	
	public static void main(String[] args) throws Exception { 
//		recognition();
		
//		inputDir = "/Users/wkerr/Sync/data/handwriting/wes/lisp/original/";
//		RecognitionExperiment exp = new RecognitionExperiment("wes-pen", 90, false);

		if (args.length != 2) { 
			System.out.println("Usage: RecognitionExperiment <path> <prefix>");
			return;

		}
		
		System.out.println("Running: " + args[0] + " -- " + args[1]);
		inputDir = args[0];
		RecognitionExperiment exp = new RecognitionExperiment(args[1], 80, false);
		exp.recognitionLearningCurve(100);
		exp.done();
		
//		trace("/tmp/signatures/wes-pen-a.xml", "wes-pen-o");
	}
	
	public static void recognition() throws Exception { 
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		String prefix = "wes-pen";
		int nFolds = 5;

		Map<String,List<Instance>> dataMap = Utils.load(inputDir, prefix, SequenceType.allen);
		Map<String,RecognizerStatistics> map = new HashMap<String,RecognizerStatistics>();
		for (String className : dataMap.keySet())
			map.put(className, new RecognizerStatistics(className));

		CrossValidation cv = new CrossValidation(nFolds);
		List<Map<String,List<Instance>>> foldsMap = cv.partition(System.currentTimeMillis(), dataMap);
		
		BufferedWriter pOut = new BufferedWriter(new FileWriter(inputDir + "recognition-performance.csv"));
		pOut.write("id,name,r_name,accepted,tp,fp,tn,fn\n");
		
		BufferedWriter sOut = new BufferedWriter(new FileWriter(inputDir + "recognition-samples.csv"));
		sOut.write("id,name,r_name,time_step,index,depth,ratio\n");
		
		// for debugging purposes... only stepping through one of the folds....
//		for (int i = 0; i < 1; ++i) { 
		for (int i = 0; i < nFolds; i++) {
			System.out.println("Fold..." + i);
			// build the training dataset in order to construct signatures
			Map<String,List<Instance>> training = new HashMap<String,List<Instance>>();
			Map<String,List<Instance>> testing = foldsMap.get(i);
			for (int j = 0; j < nFolds; ++j) { 
				if (i == j) 
					continue;
				
				for (String key : foldsMap.get(j).keySet()) { 
					List<Instance> list = training.get(key);
					if (list == null) { 
						list = new ArrayList<Instance>();
						training.put(key, list);
					}
					list.addAll(foldsMap.get(j).get(key));
				}
			}
			
			BatchSignatures bs = new BatchSignatures(training, true, 3);
			Map<String,FSMRecognizer> rMap = bs.makeRecognizers(Recognizer.cave, 95);
			
			System.out.println("  Training complete...");
			bs.writeRecognizers("/tmp/recognizers/");
			bs.writeSignatures("/tmp/signatures/");
			
			for (String className : testing.keySet()) {
				for (Instance instance : testing.get(className)) { 
					RecognizeCallable rr = new RecognizeCallable(rMap, instance);
					rr.call();
					
					// for each recognizer, let's output the data to each file.
					for (String key : rMap.keySet()) { 
						// First let's deal with the performance data
						pOut.write(rr.id() + "," + rr.name() + "," + key);

						boolean accept = rr.recognized(key);
						pOut.write("," + (accept ? 1 : 0));
						pOut.write("," + (accept && key.equals(rr.name()) ? 1 : 0));
						pOut.write("," + (accept && !key.equals(rr.name()) ? 1 : 0));
						pOut.write("," + (!accept && !key.equals(rr.name()) ? 1 : 0));
						pOut.write("," + (!accept && key.equals(rr.name()) ? 1 : 0));
						pOut.write("\n");

						// Now let's write out the detailed information about each
						// test instance.
						List<Double> depth = rr.depths(key);
						List<Double> ratio = rr.depthRatios(key);
						int start = rr.start();

						for (int j = 0; j < depth.size(); ++j) { 
							sOut.write(rr.id() + "," + rr.name() + "," + key + ",");
							sOut.write((start+j) + "," + j + "," + depth.get(j) + "," + ratio.get(j) + "\n");
						}
					}
				}
			}
		}
		
		pOut.close();
		sOut.close();

//		for (String className : map.keySet()) { 
//			RecognizerStatistics rs = map.get(className);
//			logger.debug("--- " + className);
//			logger.debug("        -- " + rs.tp() + "\t" + rs.fp());
//			logger.debug("        -- " + rs.fn() + "\t" + rs.tn());
//			logger.debug("      precision: " + rs.precision());
//			logger.debug("      recall: " + rs.recall());
//			logger.debug("      f-measure: " + rs.fscore());
//		}

		execute.shutdown();
	}
	
	public void recognition(int batch, Map<String,List<Instance>> dataMap, double splitPct) throws Exception { 
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);

		SplitAndTest sat = new SplitAndTest(0, splitPct);
		sat.partition(new Random(System.currentTimeMillis()), dataMap);

		// for this round, we need to make signatures and ultimately recognizers
		Map<String,List<Instance>> training = sat.training();
		Map<String,List<Instance>> testing = sat.testing();
		
		BatchSignatures bs = new BatchSignatures(training, true, 3);
		Map<String,FSMRecognizer> rMap = bs.makeRecognizers(Recognizer.cave, _prunePct);
		
		for (String className : testing.keySet()) {
			for (Instance instance : testing.get(className)) { 
				RecognizeCallable rr = new RecognizeCallable(rMap, instance);
				rr.call();
				
				// for each recognizer, let's output the data to each file.
				for (String key : rMap.keySet()) { 
					// First let's deal with the performance data
					_pOut.write(batch + "," + splitPct + "," + rr.id() + "," + rr.name() + "," + key);

					boolean accept = rr.recognized(key);
					_pOut.write("," + (accept ? 1 : 0));
					_pOut.write("," + (accept && key.equals(rr.name()) ? 1 : 0));
					_pOut.write("," + (accept && !key.equals(rr.name()) ? 1 : 0));
					_pOut.write("," + (!accept && !key.equals(rr.name()) ? 1 : 0));
					_pOut.write("," + (!accept && key.equals(rr.name()) ? 1 : 0));
					_pOut.write("\n");

					// Now let's write out the detailed information about each
					// test instance.
					if (_sOut != null) { 
						List<Double> depth = rr.depths(key);
						List<Double> ratio = rr.depthRatios(key);
						int start = rr.start();

						for (int j = 0; j < depth.size(); ++j) { 
							_sOut.write(batch + "," + splitPct + "," + rr.id() + "," + rr.name() + "," + key + ",");
							_sOut.write((start+j) + "," + j + "," + depth.get(j) + "," + ratio.get(j) + "\n");
						}
					}
				}
			}
		}

//		for (String className : map.keySet()) { 
//			RecognizerStatistics rs = map.get(className);
//			logger.debug("--- " + className);
//			logger.debug("        -- " + rs.tp() + "\t" + rs.fp());
//			logger.debug("        -- " + rs.fn() + "\t" + rs.tn());
//			logger.debug("      precision: " + rs.precision());
//			logger.debug("      recall: " + rs.recall());
//			logger.debug("      f-measure: " + rs.fscore());
//		}

		execute.shutdown();
	}
	
	public void recognitionLearningCurve(int repeat) throws Exception { 
		Map<String,List<Instance>> dataMap = Utils.load(inputDir, _prefix, SequenceType.allen);
		double[] pcts = new double[] { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9 };
//		double[] pcts = new double[] { 0.2, 0.4, 0.6, 0.8 };
		
		for (double pct : pcts) { 
			for (int i = 0; i < repeat; ++i) { 
				recognition(i, dataMap, pct);
			}
		}
		
		done();
	}
	
	public static void trace(String signatureFile, String dataFile) { 
		String prefix = "wes-pen";
		int nFolds = 5;

		Map<String,List<Instance>> dataMap = Utils.load(inputDir, dataFile, SequenceType.allen); 
		CrossValidation cv = new CrossValidation(nFolds);
		List<Map<String,List<Instance>>> foldsMap = cv.partition(System.currentTimeMillis(), dataMap);
		
		Map<String,List<Instance>> training = new HashMap<String,List<Instance>>();
		Map<String,List<Instance>> testing = foldsMap.get(0);
		for (int j = 1; j < nFolds; ++j) { 
			for (String key : foldsMap.get(j).keySet()) { 
				List<Instance> list = training.get(key);
				if (list == null) { 
					list = new ArrayList<Instance>();
					training.put(key, list);
				}
				list.addAll(foldsMap.get(j).get(key));
			}
		}

		
		BatchSignatures bs = new BatchSignatures(training, true, 3);
		bs.makeRecognizers(Recognizer.cave, 95);
		bs.writeSignatures(inputDir + "signatures/", 95);
		
		Map<String,FSMRecognizer> recognizers = bs.recognizers();
		
		for (String key : testing.keySet()) { 
			System.out.println("Key: " + key);
			for (Instance instance : testing.get(key)) {
				boolean accept = recognizers.get(key).test(instance.intervals());
				System.out.println("  Instance: " + instance.name() + " " + instance.id() + " -- " + accept);
			}
		}
	}
}

class RecognitionStats { 
	public Instance testInstance;
	
	
}