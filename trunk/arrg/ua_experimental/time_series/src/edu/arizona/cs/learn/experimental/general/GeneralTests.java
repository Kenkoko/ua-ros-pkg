package edu.arizona.cs.learn.experimental.general;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.XMLUtils;

public class GeneralTests {

	public static void main(String[] args) { 
//		approach();
//		jumpOver();
//		doit("ww3d");
		
		tests();
	}
	
	public static Signature makeSignature(String key, List<Instance> instances) { 
		Signature s = new Signature(key, Similarity.cosine);
		
		for (int i = 0; i < instances.size(); ++i) { 
			s.update(instances.get(i).sequence());
		}
		s.toXML("data/raw-data/handwriting/" + key + "-signature.xml");
		
		return s;
	}
	
	public static void tests() { 
		List<Instance> a = XMLUtils.loadXML("data/raw-data/handwriting/wes/xml/a.xml");
		makeSignature("a", a);
				
		List<Instance> b = XMLUtils.loadXML("data/raw-data/handwriting/wes/xml/b.xml");
		makeSignature("b", b);

		List<Instance> c= XMLUtils.loadXML("data/raw-data/handwriting/wes/xml/c.xml");
		makeSignature("c", c);
		
		SummaryStatistics aa = test(a, a);
		SummaryStatistics bb = test(b, b);
		SummaryStatistics cc = test(c, c);

		SummaryStatistics ab = test(a, b);
		SummaryStatistics ac = test(a, c);
		SummaryStatistics bc = test(b, c);
		
		
		System.out.println(" a -- a " + aa.getMean() + " -- " + aa.getStandardDeviation());
		System.out.println(" b -- b " + bb.getMean() + " -- " + bb.getStandardDeviation());
		System.out.println(" c -- c " + cc.getMean() + " -- " + cc.getStandardDeviation());

		System.out.println(" a -- b " + ab.getMean() + " -- " + ab.getStandardDeviation());
		System.out.println(" a -- c " + ac.getMean() + " -- " + ac.getStandardDeviation());
		System.out.println(" b -- c " + bc.getMean() + " -- " + bc.getStandardDeviation());
		
		
	}
	
	public static SummaryStatistics test(List<Instance> a, List<Instance> b) { 
		Params p = new Params();
		p.setMin(0, 0);
		p.setBonus(1, 1);
		p.normalize = Normalize.none;
		p.similarity = Similarity.cosine;
		
		SummaryStatistics ss = new SummaryStatistics();
		for (int i = 0; i < a.size(); ++i) { 
			for (int j = 0; j < b.size(); ++j) { 
				p.seq1 = a.get(i).sequence();
				p.seq2 = b.get(i).sequence();
				
				Report report = GeneralAlignment.alignCheckp(p);
				ss.addValue(report.score);
			}
		}
		return ss;
	}
	
	public static void buildSignature() { 
		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/chpt1-approach.lisp"));
		Map<Integer,List<Symbol>> instanceMap = new TreeMap<Integer,List<Symbol>>();
		
		Set<String> propSet = new TreeSet<String>();
		for (List<Interval> list : map.values()) { 
			for (Interval interval : list)
				propSet.add(interval.name);
		}
		List<String> props = new ArrayList<String>(propSet);
		for (Integer key : map.keySet()) {
			instanceMap.put(key, Utils.toSequence(props, BPPFactory.compress(map.get(key), Interval.eff)));
			System.out.println("Key: " + key + " ---- " + instanceMap.get(key));
		}		

		
		GeneralSignature signature = new GeneralSignature("chtp1-approach", Similarity.tanimoto);
	}

	
	/**
	 * This test involves loading in the appraoch 
	 * data and performing alignments to determine how the
	 * process will work.
	 */
	public static void approach() { 
		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/chpt1-approach.lisp"));
		Map<Integer,List<Symbol>> timelineMap = new TreeMap<Integer,List<Symbol>>();
		
		Set<String> propSet = new TreeSet<String>();
		for (List<Interval> list : map.values()) { 
			for (Interval interval : list)
				propSet.add(interval.name);
		}
		List<String> props = new ArrayList<String>(propSet);
		for (Integer key : map.keySet()) {
			timelineMap.put(key, Utils.toSequence(props, BPPFactory.compress(map.get(key), Interval.eff)));
			System.out.println("Key: " + key + " ---- " + timelineMap.get(key));
		}
		
		List<Integer> keys = new ArrayList<Integer>(map.keySet());
		for (int i = 0; i < keys.size(); ++i) { 
			Integer key1 = keys.get(i);
			List<Symbol> sequence1 = timelineMap.get(key1);
			
			for (int j = i+1; j < keys.size(); ++j) { 
				Integer key2 = keys.get(j);
				List<Symbol> sequence2 = timelineMap.get(key2);
				
				Params params = new Params(sequence1, sequence2);
				params.setMin(0, 0);
				params.setBonus(1, 1);
				params.normalize = Normalize.none;
				params.similarity = Similarity.tanimoto;
				
				Report r = GeneralAlignment.alignCheckp(params);
				r.print();
			}
		}
		
		// Let's build a pretend signature ....
		List<Symbol> signature = timelineMap.get(keys.get(0));
		for (int i = 1; i < keys.size(); ++i) { 
			List<Symbol> seq = timelineMap.get(keys.get(i));
			
			Params params = new Params(signature, seq);
			params.setMin(0, 0);
			params.setBonus(1, 1);
			
			Report r = GeneralAlignment.alignCheckp(params);
			signature = GeneralAlignment.combineAlignments(r.results1, r.results2);
		}
		
		for (Symbol s : signature)
			System.out.println("..." + s + "----" + s.weight());
	}
	
	public static void jumpOver() { 
		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/ww3d-jump-over.lisp"));
		Map<Integer,List<Symbol>> timelineMap = new TreeMap<Integer,List<Symbol>>();
		
		Set<String> propSet = new TreeSet<String>();
		for (List<Interval> list : map.values()) { 
			for (Interval interval : list)
				propSet.add(interval.name);
		}
		List<String> props = new ArrayList<String>(propSet);
		for (Integer key : map.keySet()) {
			timelineMap.put(key, Utils.toSequence(props, BPPFactory.compress(map.get(key), Interval.eff)));
		}
		
		// Let's build a pretend signature ....
		List<Integer> keys = new ArrayList<Integer>(map.keySet());
		List<Symbol> signature = timelineMap.get(keys.get(0));
		for (int i = 1; i < keys.size(); ++i) { 
			List<Symbol> seq = timelineMap.get(keys.get(i));
			
			Params params = new Params(signature, seq);
			params.setMin(0, 0);
			params.setBonus(1, 1);
			
			Report r = GeneralAlignment.alignCheckp(params);
			signature = GeneralAlignment.combineAlignments(r.results1, r.results2);
		}
		
		for (Symbol s : signature)
			System.out.println("..." + s + " ---- " + s.weight());
	}
	
	public static void doit(String prefix) { 
		Map<String,List<List<Interval>>> eMap = new HashMap<String,List<List<Interval>>>();
		Set<String> propSet = new TreeSet<String>();
		for (File f : new File("data/input/").listFiles()) {
			if ((f.getName().startsWith(prefix)) && (f.getName().endsWith("lisp"))) {
				String name = f.getName().substring(0, f.getName().indexOf(".lisp"));
				eMap.put(name, new ArrayList<List<Interval>>());
				
				Map<Integer,List<Interval>> map = Utils.load(f);
				for (List<Interval> list : map.values()) { 
					eMap.get(name).add(list);
					for (Interval interval : list)
						propSet.add(interval.name);
				}
			}
		}
		System.out.println("eMap: " + eMap.size());

		List<String> props = new ArrayList<String>(propSet);
		Map<String,List<List<Symbol>>> sMap = new HashMap<String,List<List<Symbol>>>();
		for (String key : eMap.keySet()) { 
			sMap.put(key, new ArrayList<List<Symbol>>());
			for (List<Interval> list : eMap.get(key)) { 
				sMap.get(key).add(Utils.toSequence(props, BPPFactory.compress(list, Interval.eff)));
			}
		}
		
		System.out.println("eMap: " + eMap.size() + " sMap: " + sMap.size());
		Map<String,SummaryStatistics> summaryMap = new HashMap<String,SummaryStatistics>();
		for (String key : sMap.keySet())
			summaryMap.put(key, new SummaryStatistics());
		
		for (int i = 0; i < 50; ++i) { 
			System.out.println("Iteration " + i);

			SummaryStatistics overall = new SummaryStatistics();
			Map<String,Double> map1 = singleTest(sMap);
			for (String key : map1.keySet()) {
				System.out.println("  " + key + " -- " + map1.get(key));
				summaryMap.get(key).addValue(map1.get(key));
				overall.addValue(map1.get(key));
			}
			System.out.println(" --- overall: " + overall.getMean());
		}
		
		System.out.println("Averaged");
		SummaryStatistics overall = new SummaryStatistics();
		for (String key : summaryMap.keySet()) {
			System.out.println("  " + key + " -- " + summaryMap.get(key).getMean());
			overall.addValue(summaryMap.get(key).getMean());
		}
		System.out.println(" ----- overall: " + overall.getMean());
	}
	
	public static Map<String,Double> singleTest(Map<String,List<List<Symbol>>> data) { 
		Map<String,Double> map = new HashMap<String,Double>();

		Map<String,List<List<Symbol>>> trainMap = new HashMap<String,List<List<Symbol>>>();
		Map<String,List<List<Symbol>>> testMap = new HashMap<String,List<List<Symbol>>>();
		
		for (String key : data.keySet()) { 
			map.put(key, 0.0);
			
			List<List<Symbol>> list = new LinkedList<List<Symbol>>(data.get(key));
			Collections.shuffle(list);
			
			int trainSize = (list.size() * 2) / 3;
			trainMap.put(key, new ArrayList<List<Symbol>>());
			for (int i = 0; i < trainSize; ++i) { 
				trainMap.get(key).add(list.remove(0));
			}
			
			testMap.put(key, new ArrayList<List<Symbol>>(list));
		}
		
		for (String key : testMap.keySet()) { 
			List<List<Symbol>> testSet = testMap.get(key);
			
			for (List<Symbol> instance : testSet) { 
				// now determine the distance between this instance
				// and all of the other instances in the training set
				
				double min = Double.POSITIVE_INFINITY;
				String minClass = null;
				for (String className : trainMap.keySet()) {
					for (List<Symbol> i2 : trainMap.get(className)) { 
						Params params = new Params(instance, i2);
						params.setMin(0, 0);
						params.setBonus(1, 1);
						Report report = GeneralAlignment.alignCheckp(params);
						if (report.score < min) { 
							min = report.score;
							minClass = className;
						}
					}
				}
				
				// if we guessed correctly, then increment the counter
				if (minClass.equals(key)) { 
					map.put(key, map.get(key)+1);
				}
			}
		}
		
		// now we need to normalize the scores....
		for (String key : testMap.keySet()) { 
			map.put(key, map.get(key) / (double) testMap.get(key).size());
		}
		
		return map;
	}
}

