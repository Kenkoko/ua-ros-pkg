package edu.arizona.verbs.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import org.apache.commons.collections15.bag.HashBag;

public class ProbUtils {
	
	public static<T> T sample(Map<T, Double> dist) {
		Vector<T> things = new Vector<T>();
		for (T thing : dist.keySet()) { // Put them in a fixed order
			things.add(thing);
		}
		
		Vector<Double> probs = new Vector<Double>();
		double mass = 0;
		for (T thing : things) {
			double prob = dist.get(thing);
			mass += prob;
			probs.add(mass); 
			
		}
		probs.set(probs.size()-1, 1.00000001); // Setting the last to 1 just in case
		
		double random = Math.random();
		for (int i = 0; i < probs.size(); i++) { 
			if (random > (i == 0 ? 0 : probs.get(i-1)) && random < probs.get(i)) {
				return things.get(i);
			}
		}
		System.err.println("SAMPLE DIST FAILED");
		return things.get(0);
	}
	
	// TODO: Why isn't this just Map?
	public static<T> HashMap<T, Double> computeProbDist(HashMap<T, Integer> counts) {
		double total = 0.0;
		for (Integer count : counts.values()) {
			total += count;
		}
		
		HashMap<T, Double> probs = new HashMap<T, Double>();
		for (T key : counts.keySet()) {
			probs.put(key, counts.get(key) / total);
		}
		
		return probs;
	}
	
	public static<T> HashMap<T, Double> computeProbDist(HashBag<T> counts) {
		double total = 0.0;
		for (T thing : counts.uniqueSet()) {
			total += counts.getCount(thing);
		}
		
		HashMap<T, Double> probs = new HashMap<T, Double>();
		for (T thing : counts.uniqueSet()) {
			probs.put(thing, counts.getCount(thing) / total);
		}
		
		return probs;
	}
	
	public static<T> void printDist(Map<T, Double> dist) {
		for (T thing : dist.keySet()) {
			System.out.println(thing + ": " + dist.get(thing));
		}
	}
	
	public static void main(String[] args) {
		HashMap<String, Double> testDist = new HashMap<String, Double>();
		testDist.put("a", 0.5);
		testDist.put("b", 0.3);
		testDist.put("c", 0.2);
		
		HashMap<String, Integer> countHash = new HashMap<String, Integer>();
		countHash.put("a", 0);
		countHash.put("b", 0);
		countHash.put("c", 0);
		
		for (int i = 0; i < 1000; i++) {
			String result = sample(testDist);
			countHash.put(result, countHash.get(result)+1);
		}
		
		System.out.println("a: " + countHash.get("a"));
		System.out.println("b: " + countHash.get("b"));
		System.out.println("c: " + countHash.get("c"));
	}
}
