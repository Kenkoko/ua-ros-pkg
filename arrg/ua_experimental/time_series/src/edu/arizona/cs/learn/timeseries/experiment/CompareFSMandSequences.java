package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.util.Utils;

public class CompareFSMandSequences {

	
	public static void main(String[] args) throws Exception { 
		test("ww3d", "jump-over", 0.8);
	}
	
	public static void test(String prefix, String className, double pct) throws Exception {
		File dataFile = new File("data/input/" + prefix + "-" + className + ".lisp");
		Map<Integer,List<Interval>> map = Utils.load(dataFile);

		List<List<Interval>> training = new ArrayList<List<Interval>>(map.values());
		List<List<Interval>> testing = new ArrayList<List<Interval>>();
		
		Collections.shuffle(training);

		int size = (int) Math.floor((double) training.size() * pct);
		while (training.size() > size) { 
			testing.add(training.remove(training.size()-1));
		}

		System.out.println("Training size: " + training.size());
		System.out.println("Testing size: " + testing.size());
		
		// Now we build a signature out of the training episodes
		Signature s = new Signature(className);
		for (int i = 1; i <= training.size(); ++i) { 
			List<Interval> episode = training.get(i-1);
			List<Symbol> sequence = SequenceType.allen.getSequence(episode);
			
			s.update(sequence);
			if (i % 10 == 0) {
				s = s.prune(3);
			}
		}

		// The parameters that will remain constant throughout
		// the testing phase.
		Params params = new Params();
		params.setMin(5, 0);
		params.setBonus(1.0D, 0.0D);
		params.setPenalty(-1.0D, 0.0D);

		BufferedWriter out = new BufferedWriter(new FileWriter("logs/fsm-test.csv"));
		out.write("test,time_step,accept,distance\n");
		FSMRecognizer recognizer = Recognizer.cave.build("jump-over", s, 80, false);
		for (int i = 0; i < testing.size(); ++i) { 
			List<Interval> episode = testing.get(i);
			
			// Determine the start and end points of this episode
			int start = Integer.MAX_VALUE;
			int end = 0;
			for (Interval interval : episode) {
				start = Math.min(start, interval.start);
				end = Math.max(end, interval.end);
			}

			List<BPPNode> active = new ArrayList<BPPNode>();
			active.add(recognizer.getStartState());

			// build a list of all of the time steps when the FSM 
			// accepts the test episode.
			List<Boolean> acceptList = new ArrayList<Boolean>();
			for (int j = start; j < end; j++) {
				Set<String> props = new HashSet<String>();
				for (Interval interval : episode) {
					if (interval.on(j)) 
						props.add(interval.name);
				}

				acceptList.add(recognizer.update(active, props, false));
			}
			
			Collections.sort(episode, Interval.eff);

			List<Double> distanceList = new ArrayList<Double>();
			LinkedList<Interval> list = new LinkedList<Interval>(episode);
			List<Interval> runningEpisode = new ArrayList<Interval>();
			double distance = -1;
			while (!list.isEmpty()) { 
				Interval first = list.removeFirst();
				
				runningEpisode.add(first);
				while (!list.isEmpty() && list.getFirst().end == first.end) { 
					runningEpisode.add(list.removeFirst());
				}
				
				if (runningEpisode.size() >= 2) { 
					// now determine the distance only if the number of intervals have grown
					params.seq1 = s.signature();
					params.min1 = (int) Math.round(s.trainingSize() * 0.80);
					params.seq2 = SequenceType.allen.getSequence(runningEpisode);
					distance = GeneralAlignment.distance(params);
				}
				
				// now add the distance multiple times to the list
				while (distanceList.size() < first.end) { 
					distanceList.add(distance);
				}
			}
			
			
			for (int j = 0; j < acceptList.size(); ++j) { 
				out.write(i + "," + j + "," + (acceptList.get(j) == true ? 1 : -1) + "," + distanceList.get(j) + "\n");
			}
			System.out.println("--- " + acceptList.size() + " --- " + acceptList);
			System.out.println("--- " + distanceList.size() + " --- " + distanceList);
		}
		
		out.close();
	}
}
