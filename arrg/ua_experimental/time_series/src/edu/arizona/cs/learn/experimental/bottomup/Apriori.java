package edu.arizona.cs.learn.experimental.bottomup;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

public class Apriori {

	
	private int _numEpisodes;
	private double _minSupport;
	
	private Map<Integer,List<Interval>> _episodeMap;
	private Map<Integer,List<Pattern>> _frequentMap;
	private Map<String,Pattern> _patterns;

	public Apriori() { 
		
	}
	
	/**
	 * Do level 1 since it is a base case.
	 */
	private void init() { 
		
		// Add all of the initial patterns and 
		// then add the frequent ones to the correct map...
		for (Integer key : _episodeMap.keySet()) { 
			for (Interval interval : _episodeMap.get(key)) { 
				IntervalSet is = new IntervalSet();
				is.add(interval);
				
				String name = Pattern.getName(interval);
				Pattern p = _patterns.get(name);
				if (p == null) { 
					p = new Pattern(name, 1);
					_patterns.put(name, p);
				}
				p.add(key, is);
			}
		}
		List<Pattern> frequent = findFrequent();
		_frequentMap.put(1, frequent);
		
		// Now we grow all of frequent order two patterns
		_patterns.clear();
		for (int episodeId : _episodeMap.keySet()) { 
			List<IntervalSet> sets = new ArrayList<IntervalSet>();
			for (Pattern p : frequent) { 
				List<IntervalSet> set = p.getExamples(episodeId);
				if (set != null)
					sets.addAll(set);
			}
			
			List<IntervalSet> newSets = generate(sets, 2);
			for (IntervalSet is : newSets) { 
				String name = Pattern.getName(is.intervals());
				Pattern p = _patterns.get(name);
				if (p == null) { 
					p = new Pattern(name, 2);
					_patterns.put(name, p);
				}
				p.add(episodeId, is);
			}
		}
		_frequentMap.put(2, findFrequent());
	}
	
	/**
	 * Iterate over all of the patterns currently stored in the map of 
	 * patterns and determine which of them are frequent.  Return
	 * that set of frequent patterns.
	 * @return
	 */
	private List<Pattern> findFrequent() { 
		List<Pattern> frequent = new ArrayList<Pattern>();
		List<Pattern> remove = new ArrayList<Pattern>();
		for (Pattern p : _patterns.values()) { 
			double support = (double) p.episodeCount() / (double) _numEpisodes;
			if (support > _minSupport) {
				frequent.add(p);
			} else { 
				remove.add(p);
			}
		}
		
		System.out.println("Patterns: " + _patterns.size() + " -- Frequent: " + frequent.size());
//		for (Pattern p : frequent) {
//			System.out.println("  " + p.name());
//		}
		
		return frequent;
	}
	
	/**
	 * 
	 * @param sets
	 * @param k
	 * @return
	 */
	private List<IntervalSet> generate(List<IntervalSet> sets, int k) { 
		List<IntervalSet> results = new ArrayList<IntervalSet>();
		
		Collections.sort(sets, IntervalSet.eff);
		for (int i = 0; i < sets.size(); ++i) { 
			IntervalSet is1 = sets.get(i);
			
			for (int j = i+1; j < sets.size(); ++j) { 
				IntervalSet is2 = sets.get(j);
				
				// First ensure that we will generate an IntervalSet
				// with the correct size
				if (is1.size() + is2.size() != k)
					continue;
				
				// Next ensure that the interval sets have exactly 0 overlap
				if (is1.contains(is2))
					continue;
				
				// Do these two IntervalSets interact....
				if (!Utils.LIMIT_RELATIONS || is2.start() - is1.end() < Utils.WINDOW) { 
					results.add(new IntervalSet(is1, is2));
				}
			}
		}
		
		return results;
	}
	
	/**
	 * Each set has a fixed size (greater than 1).  
	 * @param sets
	 * @return
	 */
	private List<IntervalSet> generate(List<IntervalSet> sets) { 
//		for (IntervalSet set : sets) { 
//			System.out.println("***** " + set.intervals().toString());
//		}
		
		Map<String,IntervalSet> map = new HashMap<String,IntervalSet>();
		for (int i = 0; i < sets.size(); ++i) { 
			IntervalSet is1 = sets.get(i);
			
			for (int j = i+1; j < sets.size(); ++j) { 
				IntervalSet is2 = sets.get(j);

				if (!IntervalSet.candidate(is1, is2)) 
					continue;
				
				// Do these two IntervalSets interact....
				// This should always be true....
				if (!Utils.LIMIT_RELATIONS || is2.start() - is1.end() < Utils.WINDOW) { 
					IntervalSet merge = new IntervalSet(is1, is2);
					String name = merge.intervals().toString();
					if (!map.containsKey(name)) { 
						map.put(name, merge);
//						System.out.println("Adding: " + merge.intervals().toString());
//						System.out.println("....." + is1.intervals().toString());
//						System.out.println("....." + is2.intervals().toString());
					}
				} else { 
					throw new RuntimeException("ERROR - " + is1.intervals() + " ..... " + is2.intervals());
				}
			}
		}
		
		return new ArrayList<IntervalSet>(map.values());
	}
	
	/**
	 * Grow the next level, k.  Grab all of the frequent 1 patterns
	 * as well as all of the frequent k-1 patterns and determine
	 * the frequent k patterns.
	 * @param k
	 */
	private void grow(int k) { 
		if (k <= 2)  
			throw new RuntimeException("k must be larger than two");

		_patterns.clear();
		
		List<Pattern> k1Patterns = _frequentMap.get(k-1);
		for (int episodeId : _episodeMap.keySet()) { 
			System.out.println("Starting episode " + episodeId);
			List<IntervalSet> sets = new ArrayList<IntervalSet>();
			for (Pattern p : k1Patterns) {
				List<IntervalSet> set = p.getExamples(episodeId);
				if (set != null)
					sets.addAll(set);
			}
			
			List<IntervalSet> newSets = generate(sets);
			for (IntervalSet is : newSets) { 
				String name = Pattern.getName(is.intervals());
				Pattern p = _patterns.get(name);
				if (p == null) { 
					p = new Pattern(name, 2);
					_patterns.put(name, p);
				}
				p.add(episodeId, is);
			}
			System.out.println("  Patterns: " + _patterns.size());
		}
		_frequentMap.put(k, findFrequent());
	}
	
	/**
	 * Prepare the episode map....
	 * @param file
	 */
	public void apriori(String file, double minSupport) { 
		_minSupport = minSupport;
		
		_patterns = new HashMap<String,Pattern>();
		_frequentMap = new HashMap<Integer,List<Pattern>>();
		_episodeMap = Utils.load(new File(file));
		_numEpisodes = _episodeMap.keySet().size();

		init();
		grow(3);
		grow(4);
		grow(5);
		grow(6);
	}
	
	public static void main(String[] args) { 
		Apriori apriori = new Apriori();
//		apriori.apriori("data/input/chpt1-approach.lisp", 0.5);		
		apriori.apriori("data/input/ww3d-jump-over.lisp", 0.5);
//		apriori.apriori("data/input/ww3d-jump-on.lisp", 0.5);
	}
}
