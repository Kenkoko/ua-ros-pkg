package edu.arizona.cs.learn.timeseries.independent;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class Classification {
	
	private Map<String,List<Instance>> _dataMap;
	
	public Classification() { 
		_dataMap = new HashMap<String,List<Instance>>();
	}
	
	/**
	 * 
	 * @param file
	 * @return
	 */
	public Instance load(String file) { 
		Instance instance = new Instance();
		try { 
			BufferedReader in = new BufferedReader(new FileReader(file));
			
			while (in.ready()) { 
				String line = in.readLine().replaceAll("[\"]", "");
				instance.add(line);
			}
			in.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return instance;
	}
	
	public void add(String className, String file) {
		Instance instance = load(file);
		if (!_dataMap.containsKey(className)) { 
			_dataMap.put(className, new ArrayList<Instance>());
		}
		_dataMap.get(className).add(instance);
	}
	
	/**
	 * Perform nearest neighbor classification... 
	 * Distance is determined by adding all of the variables 
	 * independently.
	 */
	public void nnClassify() { 

		Map<String,SummaryStatistics> map = new HashMap<String,SummaryStatistics>();
		for (String key : _dataMap.keySet()) { 
			map.put(key, new SummaryStatistics());
		}
		
		// Do 50 random iterations by splitting the
		// 2/3 into training data and 1/3 into test data.
		for (int i = 0; i < 50; ++i) { 
			System.out.println("Iteration " + i);
			Map<String,Double> resultMap = nnSingleTest();
			for (String key : resultMap.keySet()) { 
				System.out.println("  Key: " + key + " --- " + resultMap.get(key));
				map.get(key).addValue(resultMap.get(key));
			}
		}
		
		// now print the results
		for (String key : map.keySet()) { 
			System.out.println("Key: " + key + " : " + map.get(key).getMean());
		}
	}
	
	public Map<String,Double> nnSingleTest() { 
		Map<String,Double> map = new HashMap<String,Double>();
		
		// split each class into train and test
		Map<String,List<Instance>> trainMap = new HashMap<String,List<Instance>>();
		Map<String,List<Instance>> testMap = new HashMap<String,List<Instance>>();
		
		for (String key : _dataMap.keySet()) { 
			map.put(key, 0.0);
			
			List<Instance> list = new LinkedList<Instance>(_dataMap.get(key));
			Collections.shuffle(list);
			
			int trainSize = (list.size() * 2) / 3;
			trainMap.put(key, new ArrayList<Instance>());
			for (int i = 0; i < trainSize; ++i) { 
				trainMap.get(key).add(list.remove(0));
			}
			
			testMap.put(key, new ArrayList<Instance>(list));
		}
		
		for (String key : testMap.keySet()) { 
			List<Instance> testSet = testMap.get(key);
			
			for (Instance instance : testSet) { 
				// now determine the distance between this instance
				// and all of the other instances in the training set
				
				double min = Double.POSITIVE_INFINITY;
				String minClass = null;
				for (String className : trainMap.keySet()) {
					for (Instance i2 : trainMap.get(className)) { 
						double d = instance.distance(i2);
						if (d < min) { 
							min = d;
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
	
	public static void main(String[] args) { 
		String prefix = "data/raw-data/wes/approach-4/";

		Classification c = new Classification();
		for (int i = 1; i <= 30; ++i) { 
			c.add("class-a", prefix + "f" + i);
		}
		
		for (int i = 1; i <= 30; ++i) { 
			c.add("class-b", prefix + "g" + i);
		}
		
		c.nnClassify();
	}
	
}

class Variable { 
	public int index;
	public List<Symbol> series;
	
	public List<Symbol> sequence;
	
	public Variable(int index) {
		this.index = index;
		this.series = new ArrayList<Symbol>();
		this.sequence = null;
	}
	
	public void add(String value) { 
		series.add(new StringSymbol(value));
	}
	
	public List<Symbol> sequence() { 
		if (sequence == null) { 
			sequence = new ArrayList<Symbol>();
			sequence.addAll(series);
		}
		return sequence;
	}
	
	public double distance(Variable v) {
		Params p = new Params();
		p.seq1 = sequence();
		p.seq2 = v.sequence();
		p.setMin(0,0);
		p.setBonus(1,1);
		p.setPenalty(0,0);
		p.normalize = Normalize.knn;
		
		
		return GeneralAlignment.distance(p);
	}
}

class Instance { 
	public static final int _numVars = 6;
	public List<Variable> variables;
	
	public Instance() { 
		variables = new ArrayList<Variable>();
		for (int i = 0; i < _numVars; ++i)  
			variables.add(new Variable(i));
	}
	
	public void add(String line) { 
		String[] tokens = line.split("[ ]");
		
		for (int i = 0; i < tokens.length; ++i) { 
			variables.get(i).add(tokens[i]);
		}
	}
	
	public double distance(Instance instance) { 
//		double d = 0;
//		for (int i = 0; i < _numVars; ++i) { 
//			d += variables.get(i).distance(instance.variables.get(i));
//		}
		double d = Double.POSITIVE_INFINITY;
		for (int i = 0; i < _numVars; ++i) { 
			d = Math.min(d, variables.get(i).distance(instance.variables.get(i)));
		}
		return d;
	}
}
