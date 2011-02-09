package edu.arizona.cs.learn.timeseries.clustering;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class Distance {

	public static double[][] distances(final List<Instance> instances) { 
		final double[][] results = new double[instances.size()][instances.size()];

		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		
		List<Future<DistanceCallable>> futureList = new ArrayList<Future<DistanceCallable>>();
		for (int i = 0; i < instances.size(); ++i) { 
			for (int j = i+1; j < instances.size(); ++j) { 
				Instance i1 = instances.get(i);
				Instance i2 = instances.get(j);
//				System.out.println("Distance between " + i + "," + j + " " + i1.name() + " " + i1.id() + " and " + i2.name() + " " + i2.id());
				futureList.add(execute.submit(new DistanceCallable(i, j, i1.sequence(), i2.sequence())));
			}
		}
		
		for (Future<DistanceCallable> future : futureList) { 
			try { 
				DistanceCallable dc = future.get();
//				System.out.println("distance: " + dc.i() + " -- " + dc.j() + " score: " + dc.score());
				results[dc.i()][dc.j()] = dc.score();
				results[dc.j()][dc.i()] = dc.score();
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
		
		execute.shutdown();
		return results;
	}
}

class DistanceCallable implements Callable<DistanceCallable> {
	
	private int _i;
	private int _j;
	
	private List<Symbol> _seq1;
	private List<Symbol> _seq2;
	
	private Report _report;
	
	public DistanceCallable(int i, int j, List<Symbol> seq1, List<Symbol> seq2) {
		_i = i;
		_j = j;
		
		_seq1 = seq1;
		_seq2 = seq2;
	}
	

	@Override
	public DistanceCallable call() throws Exception {
		Params p = new Params(_seq1, _seq2);
		p.setMin(0, 0);
		p.setBonus(1, 1);
		p.setPenalty(0, 0);
		p.normalize = Normalize.signature;
		p.similarity = Similarity.strings;
		
		_report = GeneralAlignment.align(p);

		return this;
	}
	
	public int i() { 
		return _i;
	}
	
	public int j() { 
		return _j;
	}
	
	public double score() { 
		return _report.score;
	}
}
