package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
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

import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.XMLWriter;

import edu.arizona.cs.learn.timeseries.model.symbols.ComplexSymbol;
import edu.arizona.cs.learn.timeseries.model.values.Real;
import edu.arizona.cs.learn.timeseries.model.values.Value;
import edu.arizona.cs.learn.util.XMLUtils;

public class Handwriting {
	private static String PREFIX = "data/raw-data/handwriting/";
	
	public void load(String userName) throws Exception { 
		File dir = new File(PREFIX + userName + "/raw/");
		if (!dir.exists()) 
			throw new RuntimeException("Cannot find directory: " + dir.getAbsolutePath());

		Map<String,List<HWInstance>> map = new TreeMap<String,List<HWInstance>>();
		for (File f : dir.listFiles()) { 
			if (!f.getName().endsWith(".csv"))
				continue;

			// each file should contain each character of the alphabet...
			BufferedReader in = new BufferedReader(new FileReader(f));
			String header = in.readLine();

			Set<String> classNames = new HashSet<String>();
			Map<String,List<Double>> xMap = new HashMap<String,List<Double>>();
			Map<String,List<Double>> yMap = new HashMap<String,List<Double>>();
			Map<String,List<String>> strokeMap = new HashMap<String,List<String>>();
			
			while (in.ready()) { 
				String line = in.readLine();
				String[] tokens = line.split("[,]");
				
				String className = tokens[0];
				String stroke = tokens[1];
				double x = Double.parseDouble(tokens[2]);
				double y = Double.parseDouble(tokens[3]);
				
				// TODO: add in the pressure data if we aren't
				// scoring that well.... it may help distinguish
				// double pressure = Double.parseDouble(tokens[4]);
			
				if (!classNames.contains(className)) { 
					classNames.add(className);
					xMap.put(className, new ArrayList<Double>());
					yMap.put(className, new ArrayList<Double>());
					strokeMap.put(className, new ArrayList<String>());
				}
				
				xMap.get(className).add(x);
				yMap.get(className).add(y);
				strokeMap.get(className).add(stroke);
			}
			
			// Now we need to actually standardize these values 
			for (String className : classNames) { 
				xMap.put(className, TimeSeries.standardize(xMap.get(className)));
				yMap.put(className, TimeSeries.standardize(yMap.get(className)));
			}
			
			for (String className : classNames) {
				List<HWInstance> list = map.get(className);
				if (list == null) { 
					list = new ArrayList<HWInstance>();
					map.put(className, list);
				}
				list.add(new HWInstance(xMap.get(className), yMap.get(className), strokeMap.get(className)));
			}
		}
		
		for (String key : map.keySet()) { 
			List<HWInstance> instances = map.get(key);

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
	
	public static void main(String[] args) throws Exception { 
		Handwriting hw = new Handwriting();
		hw.load("wes");
	}
}

class HWInstance { 
	public List<ComplexSymbol> symbols;
	
	public List<Double> x;
	public List<Double> y;
	public List<String> stroke;
	
	public HWInstance(List<Double> x, List<Double> y, List<String> stroke) { 
		this.x = x;
		this.y = y;
		this.stroke = stroke;
		
		symbols = new ArrayList<ComplexSymbol>();
		for (int i = 0; i < x.size(); ++i) { 
			List<Value> values = new ArrayList<Value>();
			values.add(new Real("x", x.get(i)));
			values.add(new Real("y", y.get(i)));

			// technically the variable "stroke" is a symbol, but
			// for now I will treat it as real valued.
			// values.add(new Symbolic("stroke", stroke.get(i)));
			values.add(new Real("stroke", Double.parseDouble(stroke.get(i))));
			
			symbols.add(new ComplexSymbol(values, 1.0));
			
		}
	}
}
