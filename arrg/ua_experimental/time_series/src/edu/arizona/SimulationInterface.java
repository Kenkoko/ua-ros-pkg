package edu.arizona;

import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.time_series.srv.Empty;
import ros.pkg.time_series.srv.Empty.Request;
import ros.pkg.time_series.srv.Empty.Response;
import ros.pkg.time_series.msg.Episode;
import ros.pkg.time_series.srv.FindSignature;
import ros.pkg.time_series.srv.TestSignature;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.factory.SequenceFactory;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.algorithm.heatmap.HeatmapImage;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.SequenceType;

public class SimulationInterface {
	private static Logger logger = Logger.getLogger(SimulationInterface.class);

	private static HashMap<String, Signature> verbSignatures = new HashMap<String, Signature>();
	
	// For reference, see Heatmaps.java
	public static void makeHeatmap(Signature sig, List<Interval> episode, int min) {
//		s.toFile("chpt1-" + type + ".signature", min);
		String f = "/home/dhewlett/Desktop/heatmap-allen-" + sig.key() + ".png";
		HeatmapImage.makeHeatmap(f, sig.signature(), min, episode, SequenceType.allen);
	}
	
	public static List<Interval> getIntervals(Episode episode) {
		logger.debug("Getting intervals from episode...");
		List<Interval> intervals = new Vector<Interval>();
		for (int i = 0; i < episode.intervals.length; i++) {
			ros.pkg.time_series.msg.Interval interval = episode.intervals[i];
			logger.debug(interval.proposition);
			intervals.add(Interval.make(interval.proposition.toString(), interval.start, interval.end));
		}
		logger.debug("Got em.");
		return intervals;
	}
	
	public static Instance getInstance(Episode e, String name, int id) {
		List<Interval> intervals = getIntervals(e);		
		logger.debug(intervals.size());
		Collections.sort(intervals, Interval.eff);
		for (Interval i : intervals) {
			i.episode = id;
			logger.debug(i);
		}
		Instance result = new Instance(name, id, SequenceFactory.allenSequence(intervals));
		
		return result;
	}
	
	public static List<Instance> getSequences(Episode[] episodes, String name) {
		List<Instance> results = new ArrayList<Instance>();

		for (int i = 0; i < episodes.length; i++) {
			logger.debug("Converting episode " + (i+1) + " of " + episodes.length + " (" + episodes[i].intervals.length + " intervals)");
//			System.out.println(episodes[i].intervals[0]);
			results.add(getInstance(episodes[i], name, i));
		}
		
		return results;
	}

	/**
	 * @param args
	 * @throws RosException
	 */
	public static void main(String[] args) throws RosException {
		final Ros ros = Ros.getInstance();
		ros.init("time_series_node");
		NodeHandle nh = ros.createNodeHandle();

		// FindSignature service
		ServiceServer.Callback<FindSignature.Request, FindSignature.Response> scb = 
		new ServiceServer.Callback<FindSignature.Request, FindSignature.Response>() {
			public FindSignature.Response call(FindSignature.Request request) {
				logger.debug("FINDING SIGNATURE...");
				FindSignature.Response res = new FindSignature.Response();

				System.out.println("Received " + request.episodes.length + " episodes for <" + request.verb + ">");
				
				List<Instance> instances = getSequences(request.episodes, request.verb);

				Signature signature = new Signature(request.verb);
				for (Instance instance : instances) {
					signature.update(instance.sequence());
				}

				// Let's only print the relations that are in all the episodes
				Signature concensus = signature.prune(request.episodes.length - 1);
				for (WeightedObject obj : concensus.signature()) {
					System.out.println("\t" + obj.key().getKey() + " - "
							+ obj.weight());
				}
				
				verbSignatures.put(request.verb, signature);

				String filename = "signatures/" + request.verb + ".signature"; 
//				signature.toFile(filename, 0);
				signature.toXML(filename);
				
				logger.debug("Signature saved to file: " + filename);
				
				logger.debug("... DONE FINDING SIGNATURE");
				return res;
			}
		};
		ServiceServer<FindSignature.Request, FindSignature.Response, FindSignature> findService = 
			nh.advertiseService("find_signature", new FindSignature(), scb);

		System.out.println("Service " + findService.getService() + " advertised.");
		
		// TestSignature service
		ServiceServer.Callback<TestSignature.Request, TestSignature.Response> testCallback =
		new ServiceServer.Callback<TestSignature.Request, TestSignature.Response>() {
			@Override
			public TestSignature.Response call(TestSignature.Request request) {
				TestSignature.Response response = new TestSignature.Response();

				Signature sig = verbSignatures.get(request.verb);

				logger.debug("Generating Heatmap...");
				
				makeHeatmap(sig, getIntervals(request.episode), request.min);

				logger.debug("Computing Alignment Score...");
				
				Instance inst = getInstance(request.episode, "test", 0);
				Params params = new Params();
				params.setMin(request.min, 0);
				params.setBonus(1, 0);
				params.setPenalty(-1, 0);
				params.seq1 = sig.signature();
				// This overwrites the earlier call
//				params.min1 = (int) Math.round((double) sig.trainingSize() * 0.5);
				params.seq2 = inst.sequence();
				
				double alignmentScore = SequenceAlignment.distance(params);
				System.out.println("SCORE: " + alignmentScore);
				
				response.score = alignmentScore;
				
				return response;
			}
		};
		ServiceServer<TestSignature.Request, TestSignature.Response, TestSignature> testService = 
			nh.advertiseService("test_signature", new TestSignature(), testCallback);
			
		System.out.println("Service " + testService.getService() + " advertised.");
		
		ServiceServer.Callback<Empty.Request, Empty.Response> loadCallback =
		new ServiceServer.Callback<Empty.Request, Empty.Response>() {
			@Override
			public Response call(Request request) {
				Response resp = new Response();
				
				File dir = new File("signatures");

				FilenameFilter filter = new FilenameFilter() {
				    public boolean accept(File dir, String name) {
				        return name.endsWith(".signature");
				    }
				};
				
				String[] files = dir.list(filter);
				if (files == null) {
					logger.error("PROBLEM LOADING SIGNATURES");
				} else {
					logger.debug("LOADING " + files.length + " SIGNATURES...");
				    for (String file : files) {
				    	logger.debug("\t" + file);
				        Signature.fromXML("signatures/" + file);
				    }
				}
				
				logger.debug("SIGNATURES LOADED");
				
				return resp;
			}
		};
		ServiceServer<Empty.Request, Empty.Response, Empty> loadService = 
			nh.advertiseService("load_signatures", new Empty(), loadCallback);
			
		System.out.println("Service " + loadService.getService() + " advertised.");
			
		System.out.println("Initialization Complete.");
		
		ros.spin();
	}
}
