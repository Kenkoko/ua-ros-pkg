package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.XMLWriter;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.symbols.ComplexSymbol;
import edu.arizona.cs.learn.util.XMLUtils;

public class Handwriting {
	private static String PREFIX = "/Users/wkerr/Dropbox/data/handwriting/";
	
	private Map<String,List<HWInstance>> _map;
	
	public void load(String userName) throws Exception { 
		File dir = new File(PREFIX + userName + "/raw/");
		if (!dir.exists()) 
			throw new RuntimeException("Cannot find directory: " + dir.getAbsolutePath());

		_map = new TreeMap<String,List<HWInstance>>();
		for (File f : dir.listFiles()) { 
			if (!f.getName().endsWith(".csv"))
				continue;

			// each file should contain each character of the alphabet...
			BufferedReader in = new BufferedReader(new FileReader(f));
			String header = in.readLine();

			Set<String> classNames = new HashSet<String>();
			Map<String,List<Double>> xMap = new HashMap<String,List<Double>>();
			Map<String,List<Double>> yMap = new HashMap<String,List<Double>>();
						
			Map<String,List<Integer>> strokeMap = new HashMap<String,List<Integer>>();
			
			while (in.ready()) { 
				String line = in.readLine();
				String[] tokens = line.split("[,]");
				
				String className = tokens[0];
				int stroke = Integer.parseInt(tokens[1]);
				double x = Double.parseDouble(tokens[2]);
				double y = Double.parseDouble(tokens[3]);
				
				// TODO: add in the pressure data if we aren't
				// scoring that well.... it may help distinguish
				// double pressure = Double.parseDouble(tokens[4]);
			
				if (!classNames.contains(className)) { 
					classNames.add(className);
					xMap.put(className, new ArrayList<Double>());
					yMap.put(className, new ArrayList<Double>());
					
					strokeMap.put(className, new ArrayList<Integer>());
				}
				
				xMap.get(className).add(x);
				yMap.get(className).add(y);
				strokeMap.get(className).add(stroke);
			}
			
			for (String className : classNames) {
				List<HWInstance> list = _map.get(className);
				if (list == null) { 
					list = new ArrayList<HWInstance>();
					_map.put(className, list);
				}
				list.add(new HWInstance(xMap.get(className), yMap.get(className), strokeMap.get(className)));
			}
		}
	}

	/**
	 * Write out the XML file containing ComplexSymbols for the dataset.
	 * TODO: artificial time needs to be inserted between strokes.
	 * @param userName
	 * @param map
	 */
	public void writeXML(String userName) { 
		for (String key : _map.keySet()) { 
			List<HWInstance> instances = _map.get(key);

			// Write an XML document containing all of the instances
			Document document = DocumentHelper.createDocument();
			Element root = document.addElement("DataSet")
					.addAttribute("key", key)
					.addAttribute("count", instances.size()+"");


			for (HWInstance instance : instances) { 
				XMLUtils.toXML(root, instance.symbols);
			}

			try {
				String output = PREFIX + userName + "/xml/" + key + ".xml";
				OutputFormat format = OutputFormat.createPrettyPrint();
				XMLWriter writer = new XMLWriter(new FileWriter(output), format);
				writer.write(document);
				writer.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
	
	
	public void writeLisp(String userName) throws Exception { 
		for (String key : _map.keySet()) { 
			List<HWInstance> instances = _map.get(key);

			String output = "data/input/" + userName + "-pen-" + key + ".lisp";
			System.out.println("Constructing: " + output);
			System.out.println("  Num instances: " + instances.size());
			BufferedWriter out = new BufferedWriter(new FileWriter(output));

			for (int k = 0; k < instances.size(); ++k) { 
				HWInstance data = instances.get(k);

				Map<String,List<String>> map = new HashMap<String,List<String>>();
				map.put("x", new ArrayList<String>());
				map.put("y", new ArrayList<String>());
				map.put("dx", new ArrayList<String>());
				map.put("dy", new ArrayList<String>());
				map.put("pen", new ArrayList<String>());
				
				
				for (int stroke : data.strokes) { 
					map.get("x").addAll(TimeSeries.sax(data.x.get(stroke), 5));
					map.get("y").addAll(TimeSeries.sax(data.y.get(stroke), 5));
					
					map.get("dx").addAll(TimeSeries.sdl(data.dx.get(stroke)));
					map.get("dy").addAll(TimeSeries.sdl(data.dy.get(stroke)));

					List<String> tmp = map.get("pen");
					while (tmp.size() < map.get("x").size()) 
						tmp.add("down");
					
					// Now we need to add the gap in.
					for (int j = 0; j < 10; ++j) { 
						map.get("x").add("NaN");
						map.get("y").add("NaN");

						map.get("dx").add("NaN");
						map.get("dy").add("NaN");
						
						map.get("pen").add("up");
					}
				}
				
				List<Interval> intervals = new ArrayList<Interval>();
//				intervals.addAll(TimeSeries.toIntervals("x", map.get("x")));
				intervals.addAll(TimeSeries.toIntervals("x", map.get("dx")));

//				intervals.addAll(TimeSeries.toIntervals("y", map.get("y")));
				intervals.addAll(TimeSeries.toIntervals("y", map.get("dy")));
				intervals.addAll(TimeSeries.toIntervals("pen", map.get("pen")));
				
				out.write("(" + k + "\n (\n");
				for (Interval interval : intervals) { 
					out.write("  (\"" + interval.name + "\" " + interval.start + " " + interval.end + ")\n");
				}
				out.write (" )\n)\n");
			}
			out.close();
		}
	}
	
	public static void main(String[] args) throws Exception { 
		Handwriting hw = new Handwriting();
		hw.load("wes");
		hw.writeLisp("wes");
	}
}

class HWInstance { 
	public List<ComplexSymbol> symbols;
	
	public Map<Integer,List<Double>> x;
	public Map<Integer,List<Double>> dx;
	
	public Map<Integer,List<Double>> y;
	public Map<Integer,List<Double>> dy;
	
	public Set<Integer> strokes;
	
	public HWInstance(List<Double> xts, List<Double> yts, List<Integer> strokets) { 
		x = new TreeMap<Integer,List<Double>>();
		dx = new TreeMap<Integer,List<Double>>();

		y = new TreeMap<Integer,List<Double>>();
		dy = new TreeMap<Integer,List<Double>>();
		
		strokes = new TreeSet<Integer>(strokets);
		
		// First thing to do is split the time series into the individual strokes.
		for (int i = 0; i < strokets.size(); ++i) {
			int id = strokets.get(i);

			List<Double> xtmp = x.get(id);
			if (xtmp == null) {
				xtmp = new ArrayList<Double>();
				x.put(id, xtmp);
			}
			xtmp.add(xts.get(i));
			
			List<Double> ytmp = y.get(id);
			if (ytmp == null) { 
				ytmp = new ArrayList<Double>();
				y.put(id, ytmp);
			}
			ytmp.add(yts.get(i));
		}
		
		for (Integer stroke : x.keySet()) { 
			List<Double> column = TimeSeries.standardize(TimeSeries.linearFilter(x.get(stroke), 3));
			List<Double> diff = TimeSeries.diff(column);
			
			x.put(stroke, column);
			dx.put(stroke, diff);
			
			column = TimeSeries.standardize(TimeSeries.linearFilter(y.get(stroke), 3));
			diff = TimeSeries.diff(column);
			
			y.put(stroke, column);
			dy.put(stroke, diff);
		}
//		makeSymbols();
	}
	
	private void makeSymbols() { 
		throw new RuntimeException("FIX STROKE PROBLEMS FIRST!");
//		symbols = new ArrayList<ComplexSymbol>();
//		for (int i = 0; i < x.size(); ++i) { 
//			List<Value> values = new ArrayList<Value>();
//			values.add(new Real("x", x.get(i)));
//			values.add(new Real("y", y.get(i)));
//			
//			values.add(new Real("dx", dx.get(i)));
//			values.add(new Real("dy", dy.get(i)));
//
//			// technically the variable "stroke" is a symbol, but
//			// for now I will treat it as real valued.
//			// values.add(new Symbolic("stroke", stroke.get(i)));
//			values.add(new Real("stroke", Double.parseDouble(stroke.get(i))));
//			
//			symbols.add(new ComplexSymbol(values, 1.0));
//			
//		}
	}
}
