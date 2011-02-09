package edu.arizona.cs.learn.timeseries.experiment;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.algorithm.heatmap.HeatmapImage;
import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.evaluation.BatchSignatures;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.RandomFile;
import edu.arizona.cs.learn.util.Utils;

public class Visualize {

	
	public static void main(String[] args) { 
		Map<String,List<Instance>> map = Utils.load("data/input/", "global-ww2d", SequenceType.allen);
		BatchSignatures bs = new BatchSignatures(map);
		bs.makeSignatures();
		
		makeImages(bs.signatures().get("global-ww2d-collide"), map.get("global-ww2d-pass"), "/tmp/output/", 
				"global-ww2d-pass", true);
		
//		List<Interval> set1 = new ArrayList<Interval>();
//		set1.add(Interval.make("F", 0, 15));
//		set1.add(Interval.make("D", 1, 20));
//		set1.add(Interval.make("S", 15, 20));
//
//		Paint.render(set1, "/Users/wkerr/Desktop/example1.png");
		
		
//		System.out.println(SequenceType.allen.getSequence(set1));
//		
//		List<Interval> set2 = new ArrayList<Interval>();
//		set2.add(Interval.make("C", 1, 3));
//		set2.add(Interval.make("A", 3, 6));
//		set2.add(Interval.make("B", 4, 9));
//		set2.add(Interval.make("C", 6, 10));
//		System.out.println(SequenceType.allen.getSequence(set2));
//		set2.add(Interval.make("D", 0, 0));
//		Paint.render(set2, "/Users/wkerr/Desktop/example2.png");
//
//		List<Interval> set3 = new ArrayList<Interval>();
//		set3.add(Interval.make("A", 1, 5));
//		set3.add(Interval.make("B", 4, 6));
//		set3.add(Interval.make("C", 5, 7));
//		System.out.println(SequenceType.allen.getSequence(set3));
//		set3.add(Interval.make("D", 0, 0));
//		Paint.render(set3, "/Users/wkerr/Desktop/example3.png");
		
		
//		Paint.sample2();
//		niallData();
	}
	
	public static void niallData() { 
		String pid = RandomFile.getPID();
		String dir = "/tmp/niall-" + pid + "/";

		SyntheticExperiments.generateClass(pid, "f", 0, 0, 25);
		SyntheticExperiments.generateClass(pid, "g", 1.0, 0, 25);

		makeImages(dir, dir, "niall-f", false);
		makeImages(dir, dir, "niall-g", false);
	}
	
	public static void makeImages(String dataDir, String outputDir, String prefix, boolean compress) { 
		Map<Integer,List<Interval>> map = Utils.load(new File(dataDir + prefix + ".lisp"));
		if (compress) { 
			Map<Integer,List<Interval>> compressedMap = new TreeMap<Integer,List<Interval>>();
			for (Integer key : map.keySet()) { 
				compressedMap.put(key, BPPFactory.compress(map.get(key), Interval.eff));
			}
			map = compressedMap;
		}
		
		Map<Integer,List<Symbol>> instances = new TreeMap<Integer,List<Symbol>>();
		for (Integer key : map.keySet())  
			instances.put(key, SequenceType.allen.getSequence(map.get(key)));

		System.out.println("Making images...");
		// build a signature....
		List<Integer> eIds = new ArrayList<Integer>(map.keySet());
		Signature s = new Signature(prefix);
		for (int i = 1; i <= 20; ++i) { 
			System.out.println("\t...i = " + i);
			s.update(instances.get(eIds.get(i-1)));
			
			if (i % 10 == 0)
				s = s.prune(3);
		}
		System.out.println("\t...signature trained");
		
		// print all of the regular images 
		for (int id : eIds) { 
			String pre = outputDir + prefix + "-" + id;
			
			Paint.render(map.get(id), pre + ".png");
			HeatmapImage.makeHeatmap(pre + "-hm.png", s.signature(), 0, map.get(id), SequenceType.allen);
		}
	}
	
	public static void makeImages(Signature s, List<Instance> instances, 
			String outputDir, String prefix, boolean compress) { 

		for (Instance instance : instances) { 
			String file = outputDir + prefix + "-" + instance.id() + ".png";
			List<Interval> intervals = instance.intervals();
			if (compress) 
				intervals = BPPFactory.compress(intervals, Interval.eff);
			
			HeatmapImage.makeHeatmap(file, s.signature(), 0, intervals, SequenceType.allen);
		}
	}	
}
