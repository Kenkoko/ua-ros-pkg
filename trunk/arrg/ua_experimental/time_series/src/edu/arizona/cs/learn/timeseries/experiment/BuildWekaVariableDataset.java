package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

/**
 * @author wkerr
 *
 */
public class BuildWekaVariableDataset {

    private static Logger logger = Logger.getLogger(BuildWekaVariableDataset.class);
    
    public static void main(String[] args) { 
    	String prefix = "ww2d-";
    	
    	List<String> classNames = new ArrayList<String>();
    	for (File f : new File("data/input/").listFiles()) {
    		if (f.getName().startsWith(prefix)) { 
    			String name = f.getName();
    			classNames.add(name.substring(0, name.indexOf(".lisp")));
    		}
    	}
    	writeWeka(classNames, prefix.substring(0, prefix.length()-1));
    }
    
    public static void writeWeka(List<String> classNames, String output) { 
    	Set<String> variableSet = new HashSet<String>();
    	List<String> classes = new ArrayList<String>();
    	
    	Map<String,List<Set<String>>> map = new HashMap<String,List<Set<String>>>();
    	
		for (String className : classNames) { 
			classes.add(className);
			logger.debug("class name: [" + className + "]");
			
			List<Set<String>> episodes = new ArrayList<Set<String>>();
			map.put(className, episodes);
			
			Map<Integer,List<Interval>> intervalMap = Utils.load(new File("data/input/" + className + ".lisp"));
			for (Integer id : intervalMap.keySet()) { 
				Set<String> propSet = new HashSet<String>();
				episodes.add(propSet);
				
				for (Interval i : intervalMap.get(id)) { 
					variableSet.add(i.name);
					propSet.add(i.name);
				}
			}
		}

		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("data/weka/" + output + ".arff"));
			out.write("% This a simple demonstration that variables have for classification \n");
			out.write("% dataset generated for " + output + "\n");
			out.write("% from files ---- \n");
			for (String s : classNames) { 
				out.write("%   " + s + "\n");
			}

			out.write("@relation " + output + "\n\n");
			List<String> variables = new ArrayList<String>(variableSet);
			Collections.sort(variables);
			for (String v : variables) { 
				out.write("@attribute \"" + v + "\" {TRUE,FALSE}\n");
			}
			out.write("\n");
			StringBuffer buf = new StringBuffer();
			for (String c : classes) { 
				buf.append(c + ",");
			}
			buf.deleteCharAt(buf.length()-1);
			out.write("@attribute classLabel {" + buf.toString() + "}\n");
			out.write("\n");
			out.write("@data\n");
			
			for (String className : map.keySet()) { 
				List<Set<String>> episodes = map.get(className);
				for (Set<String> episode : episodes) { 
					for (String v : variables) { 
						if (episode.contains(v))
							out.write("TRUE,");
						else
							out.write("FALSE,");
					}
					out.write(className + "\n");
				}
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
    }
}
