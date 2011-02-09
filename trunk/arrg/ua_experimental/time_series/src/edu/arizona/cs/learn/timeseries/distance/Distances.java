package edu.arizona.cs.learn.timeseries.distance;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class Distances {
	
	/**
	 * Read in the instances from the files with the correct prefix.  Construct the sequences
	 * then actually build the matrix as well.
	 * @param prefix
	 * @param generator
	 * @param threads
	 * @return
	 */
	public static double[][] distances(String prefix, SequenceType type) { 
    	Map<String,List<Instance>> map = Utils.load(prefix, type);
    	
    	List<Instance> allInstances = new ArrayList<Instance>();
    	for (List<Instance> list : map.values()) { 
    		allInstances.addAll(list);
    	}
    	return distances(allInstances);
	}
	
	/**
	 * Build the distance matrix for this clustering algorithm
	 * with multiple threads to parallelize out the computation
	 * @param instances
	 * @param threads
	 * @return
	 */
	public static double[][] distances(List<Instance> instances) { 
	    ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		List<Future<DistanceResult>> future = new ArrayList<Future<DistanceResult>>();

		for (int i = 0; i < instances.size(); ++i) { 
			Instance i1 = instances.get(i);
			// remember to set the unique ids for each of the instances
			i1.uniqueId(i);
			
			for (int j = 0; j < instances.size(); ++j) { 
				Instance i2 = instances.get(j);
				future.add(execute.submit(new DistanceCallable(i1, i2, i, j)));
			}
		}
		
		double[][] matrix = new double[instances.size()][instances.size()];
		for (Future<DistanceResult> results : future) { 
			DistanceResult thread = null;
			try {
				thread = results.get();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (ExecutionException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			matrix[thread.i][thread.j] = thread.d;
			matrix[thread.j][thread.i] = thread.d;
		}
	    
		execute.shutdown();
		return matrix;
	}
	
	/**
	 * Write the given matrix out to a file so that it can be used by other
	 * algorithms to learn interesting information
	 * @param fileName
	 * @param matrix
	 */
	public static void save(String fileName, double[][] matrix) { 
    	StringBuffer buf = new StringBuffer();
    	for (int i = 0; i < matrix.length; ++i) { 
    		for (int j = 0; j < matrix[i].length; ++j) { 
    			buf.append(matrix[i][j] + ",");
    		}
    		buf.deleteCharAt(buf.length()-1);
    		buf.append("\n");
    	}
    	
    	try { 
    		BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
        	out.write(buf.toString());
        	out.close();
    	} catch (Exception e) { 
    		e.printStackTrace();
    	}	
    }
	
	public static void save(String prefix, SequenceType type) { 
		List<Instance> all = new ArrayList<Instance>();
    	for (File f : new File("data/input/").listFiles()) {
    		if (f.getName().startsWith(prefix)) { 
    			String name = f.getName();
    			String className = name.substring(0, name.indexOf(".lisp"));

    			List<Instance> list = Utils.sequences(className, f.getAbsolutePath(), type);
    			System.out.print(className + " range " + all.size() + " to ");
    			all.addAll(list);
    			System.out.println(all.size());
    		}
    	}

		Params params = new Params();
		params.normalize = Normalize.knn;
		params.setMin(0, 0);
		params.setBonus(1,1);
		params.setPenalty(0, 0);
    	
		SummaryStatistics ss = new SummaryStatistics();
    	double[][] table = new double[all.size()][all.size()];
    	for (int i = 0; i < all.size(); ++i) { 
    		Instance i1 = all.get(i);
    		
    		for (int j = i+1; j < all.size(); ++j) { 
    			Instance i2 = all.get(j);
    			
    			params.seq1 = i1.sequence();
    			params.seq2 = i2.sequence();
    			
    			// 1 - distance for affinity propogation since
    			// it larger values are deemed more similar
//    			double d = 1-SequenceAlignment.distance(params);
    			double d = GeneralAlignment.distance(params);
    			table[i][j] = d;
    			table[j][i] = d;
    			
    			ss.addValue(d);
    		}
    	}
    	
    	for (int i = 0; i < all.size(); ++i) { 
    		table[i][i] = 0; //-100;// -ss.getMean();
    	}
    	
    	StringBuffer buf = new StringBuffer();
    	for (int i = 0; i < all.size(); ++i) { 
    		for (int j = 0; j < all.size(); ++j) { 
    			buf.append(table[i][j] + ",");
    		}
    		buf.deleteCharAt(buf.length()-1);
    		buf.append("\n");
    	}
    	
    	try { 
    		BufferedWriter out = new BufferedWriter(new FileWriter("distances/" + prefix + "-" + type + ".csv"));
        	out.write(buf.toString());
        	out.close();
    	} catch (Exception e) { 
    		e.printStackTrace();
    	}
	}
	
	/**
	 * We've already written the matrix out, so it wouldn't hurt to load it in.
	 * @param file
	 * @param sequences
	 * @return
	 */
	public static double[][] load(String file, List<Instance> sequences) { 
		double[][] matrix = new double[sequences.size()][sequences.size()];
		try { 
			BufferedReader in = new BufferedReader(new FileReader(file));
			int row = 0;
			while (in.ready()) {
				String line = in.readLine();
				String[] tokens = line.split("[,]");
				
				if (tokens.length != sequences.size())
					throw new RuntimeException("Error in sizes: " + sequences.size() + " " + tokens.length);
				
				for (int i = 0; i < tokens.length; ++i) { 
					matrix[row][i] = Double.parseDouble(tokens[i]);
				}
				
				++row;
			}
			in.close();
		} catch (Exception e) { 
			
		}
		return matrix;
	}
	
	public static void main(String[] args) { 
		List<Instance> instances = Utils.sequences("trace-1", "data/input/trace-1.lisp", SequenceType.allen);
		save("/Users/wkerr/Desktop/distances.csv", distances(instances));
	}
}

class DistanceResult {
	public int i;
	public int j;
	
	public double d;
	
	public DistanceResult(int i, int j, double d) { 
		this.i = i;
		this.j = j;
		this.d = d;
	}
}

/**
 * This thread will calcuate the distance between two instances
 * given in the constructor
 * @author wkerr
 *
 */
class DistanceCallable implements Callable<DistanceResult> { 
	public int i;
	public int j;
	
	public Instance i1;
	public Instance i2;
	
	public DistanceCallable(Instance i1, Instance i2, int i, int j) { 
		this.i = i;
		this.j = j;
		
		this.i1 = i1;
		this.i2 = i2;
	}
	
	public DistanceResult call() throws Exception { 
		Params params = new Params();
		params.normalize = Normalize.knn;
		params.setMin(0, 0);
		params.setBonus(1,1);
		params.setPenalty(0,0);			
		params.seq1 = i1.sequence();
		params.seq2 = i2.sequence();
		double d = GeneralAlignment.distance(params);
		return new DistanceResult(i,j,d);
	}
}
