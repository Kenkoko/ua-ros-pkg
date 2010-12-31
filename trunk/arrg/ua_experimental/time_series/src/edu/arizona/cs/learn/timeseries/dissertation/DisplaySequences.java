package edu.arizona.cs.learn.timeseries.dissertation;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class DisplaySequences {

	public static void main(String[] args) { 
		SignatureExample.init();
		
		String file = "data/input/chpt1-approach.lisp";
		AllenRelation.text = AllenRelation.fullText;
		SequenceType type = SequenceType.allen;
		
		Map<Integer,List<Interval>> map = Utils.load(new File(file));
		for (Integer eid : map.keySet()) { 
			List<Interval> list = map.get(eid);
			Collections.sort(list, Interval.eff);
			
			// Let's print the intervals...
			for (Interval i : list) { 
				System.out.println("-------" + i);
			}
			
			List<Symbol> sequence = type.getSequence(list);
			System.out.println("Sequence: " + eid);
			for (Symbol obj : sequence) { 
				System.out.println("  " + obj.toString());
			}
		}
	}
	
}
