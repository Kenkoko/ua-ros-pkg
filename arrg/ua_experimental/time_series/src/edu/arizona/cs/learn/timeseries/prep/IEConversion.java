package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

/**
 * IE Conversion converts from the input type (our Lisp format)
 * into the output type which is used as input to the IEMiner and
 * IEClassifier programs
 * @author wkerr
 *
 */
public class IEConversion {

//	 Format of inputdatafile: 
//	    Each line represent one event sequence (i.e., one transaction). 
//	    Event sequence must be ordered wrt time.
//	    
//	    eventtype starttime endtime eventtype starttime endtime
//	    eventtype starttime endtime eventtype starttime endtime eventtype starttime endtime eventtype starttime endtime
//	    eventtype starttime endtime
//	    ...
//	    
//	    [Example of input file]:     
//	    A 1 3 B 2 4 A 8 9 D 10 12
//	    A 3 5 
//	    B 2 5 C 3 4
//	    ...
	    
	public static void convert(String inputF, String outputF, boolean mapKeys) throws Exception { 
		Map<Integer,List<Interval>> map = Utils.load(new File(inputF));
		
		Map<String,Integer> keyMap = new HashMap<String,Integer>();
		BufferedWriter out = new BufferedWriter(new FileWriter(outputF));
		for (List<Interval> list : map.values()) { 
			Collections.sort(list, Interval.esf);
			for (Interval i : list) { 
				
				if (mapKeys) {
					Integer name = keyMap.get(i.name);
					if (name == null) { 
						name = keyMap.size();
						keyMap.put(i.name, name);
					}
					
					out.write(name + " " + i.start + " " + i.end + " ");
				} else { 
					out.write(i.name + " " + i.start + " " + i.end + " ");
				}
			}
			out.write("\n");
			out.flush();
		}
		out.close();
	}
	
	/**
	 * Load in all of the different classes and convert them into a form
	 * that the IEClassifier can handle.
	 * @param prefix
	 * @throws Exception
	 */
	public static void splitAndConvert(String dataDir, String prefix, double pct, boolean mapKeys) throws Exception { 
		List<String> activities = Utils.getActivityNames(prefix);
		Map<String,List<List<Interval>>> allMap = new HashMap<String,List<List<Interval>>>();
		Map<String,Integer> keyMap = new HashMap<String,Integer>();
		for (String key : activities) { 
			Map<Integer,List<Interval>> map = Utils.load(new File(dataDir + key + ".lisp"));
			
			// Iterate over all of the intervals and add the names to the keyMap
			for (List<Interval> list : map.values()) { 
				Collections.sort(list, Interval.esf);
				for (Interval i : list) { 
					Integer name = keyMap.get(i.name);
					if (name == null) { 
						name = keyMap.size();
						keyMap.put(i.name, name);
					}
				}
			}
			
			allMap.put(key, new ArrayList<List<Interval>>(map.values()));
		}

		for (String key : allMap.keySet()) { 
			write(key, keyMap, allMap.get(key), pct, mapKeys);
		}
		
	}
	
	/**
	 * We need to create two files.  One training and one testing.
	 * @param data
	 * @param pct
	 */
	public static void write(String key, Map<String,Integer> propMap, List<List<Interval>> data, double pct, boolean mapKeys) 
				throws Exception { 
		String prefix = "/Users/wkerr/Dropbox/dhaval-app/classify/";
		
		int min = (int) Math.floor(data.size() * pct);
		Collections.shuffle(data);

		BufferedWriter out = new BufferedWriter(new FileWriter(prefix + key + "-input.data"));
		for (int i = 0; i < min; ++i) {
			List<Interval> instance = data.remove(0);
			Collections.sort(instance, Interval.esf);
			for (Interval interval : instance)  {
				if (mapKeys)
					out.write(propMap.get(interval.name) + " " + interval.start + " " + interval.end + " ");
				else
					out.write(interval.name + " " + interval.start + " " + interval.end + " ");
			}
			out.write("\n");
			out.flush();
		}
		out.close();
		
		BufferedWriter test = new BufferedWriter(new FileWriter(prefix + key + "-test.data"));
		while (!data.isEmpty()) { 
			List<Interval> instance = data.remove(0);
			Collections.sort(instance, Interval.esf);
			for (Interval interval : instance) {
				if (mapKeys) 
					test.write(propMap.get(interval.name) + " " + interval.start + " " + interval.end + " ");
				else 
					test.write(interval.name + " " + interval.start + " " + interval.end + " ");
			}
			test.write("\n");
			test.flush();
		}
		
	}
	
	public static void main(String[] args) throws Exception { 
//		convert("data/input/ww3d-jump-over.lisp", "/Users/wkerr/Dropbox/dhaval-app/data/ww3d-jump-over.data");
		splitAndConvert("/tmp/niall-9507/", "niall", 2.0/3.0, true);
	}
}
