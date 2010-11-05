package edu.arizona.verbs.util;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import edu.arizona.cs.learn.algorithm.alignment.factory.SequenceFactory;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.Relation;

public class TraceConverter {
	
	public static Instance convertTrace(List<OOMDPState> trace, String verb, Map<String, String> nameMap) {
		List<Interval> closedIntervals = new Vector<Interval>();
		Map<String, Interval> openIntervals = new HashMap<String, Interval>();

		for (int i = 0; i < trace.size(); i++) {
			OOMDPState state = trace.get(i);
			List<Relation> relations = state.getRelations();
			for (Relation relation : relations) {
				Relation remapped = Remapper.remapRelation(relation, nameMap);
				String relString = remapped.toString();
				
				if (openIntervals.containsKey(relString)) {
					if (!relation.value) { // Close an existing interval
						Interval open = openIntervals.get(relString);
						open.end = i;
						closedIntervals.add(open);
						openIntervals.remove(relString);
					}
				} else { 
					if (relation.value) { // Open a new interval
						Interval open = new Interval();
						open.name = relString;
						open.start = i;
						// Are any other attributes necessary?
						openIntervals.put(relString, open);
					} 
				}
			}
		}
		
		for (String relation : openIntervals.keySet()) {
			Interval interval = openIntervals.get(relation);
			interval.end = trace.size();
			closedIntervals.add(interval);
		}
		
		Collections.sort(closedIntervals, Interval.eff);
		// TODO: What to do for ID?
		Instance result = new Instance(verb, 0, SequenceFactory.allenSequence(closedIntervals));
		return result;
	}
	
}
