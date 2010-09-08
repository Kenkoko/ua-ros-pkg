package edu.arizona.cs.learn.timeseries.dissertation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.model.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class SampleDataset {
    private static Logger logger = Logger.getLogger(SampleDataset.class);

    public static void main(String[] args) { 
//    	drawExamplesSameSize();

		List<Interval> set = new ArrayList<Interval>();
		set.add(Interval.make("distance-decreasing(agent,box)", 1, 20));
		set.add(Interval.make("forward(agent)", 0, 11));
		set.add(Interval.make("speed-decreasing(agent)", 11, 20));
		set.add(Interval.make("collision(agent,box)", 20, 21));
		Paint.render(set, "/Users/wkerr/Desktop/states-1.png");
    }
    
    
	public static void drawExamplesSameSize() { 
		Utils.LIMIT_RELATIONS = false;
		Utils.WINDOW = 4;
		AllenRelation.MARGIN = 4;
		AllenRelation.text = AllenRelation.fullText;

		Paint._rowHeight = 10;
		Paint._timeWidth = 5;
		Paint._fontSize = 9;
		
//		  towards(agent1 box1)   011111111110
//		  forward(agent)         111111000000
//		  down(speed(agent1))    000000111111
//		  collision(agent1 box1) 000000000001
		  
		List<Interval> set = new ArrayList<Interval>();
		set.add(Interval.make("distance-decreasing(agent,box)", 1, 20));
		set.add(Interval.make("forward(agent)", 0, 11));
		set.add(Interval.make("speed-decreasing(agent)", 11, 20));
		set.add(Interval.make("collision(agent,box)", 20, 21));
		set.add(Interval.make("distance-decreasing(agent,box2)", 0, 0));
		set.add(Interval.make("distance-stable(agent,box2)", 0, 0));
		set.add(Interval.make("distance-increasing(agent,box2)", 0, 0));
		set.add(Interval.make("turn-left(agent)", 0, 0));
		set.add(Interval.make("turn-right(agent)", 0, 0));
		Paint.render(set, "/Users/wkerr/Desktop/approach-ts-1.png");
		Collections.sort(set, Interval.esf);
		List<WeightedObject> sequence = SequenceType.allen.getSequence(set);
		logger.debug("Sequence Size: " + sequence.size());	
		for (WeightedObject obj : sequence) { 
			logger.debug("\t" + obj.key());
		}
		logger.debug("\n");
		
//		  towards(agent1 box1)  01111111111
//		  forward(agent1)       11111100000
//		  down(speed(agent1))   00000011111
//		  towards(agent1 box2)  01111000000
//		  away(agent1 box2)     00000011111		
	
		set = new ArrayList<Interval>();
		set.add(Interval.make("distance-decreasing(agent,box)", 1, 16));
		set.add(Interval.make("forward(agent)", 0, 8));
		set.add(Interval.make("speed-decreasing(agent)", 8, 16));
		set.add(Interval.make("collision(agent,box)", 16, 17));
		set.add(Interval.make("distance-decreasing(agent,box2)", 1, 7));
		set.add(Interval.make("distance-stable(agent,box2)", 7, 9));
		set.add(Interval.make("distance-increasing(agent,box2)", 9, 16));
		set.add(Interval.make("turn-left(agent)", 0, 0));
		set.add(Interval.make("turn-right(agent)", 21, 21));
		Paint.render(set, "/Users/wkerr/Desktop/approach-ts-2.png");
		Collections.sort(set, Interval.esf);
		sequence = SequenceType.allen.getSequence(set);
		logger.debug("Sequence Size: " + sequence.size());	
		for (WeightedObject obj : sequence) { 
			logger.debug("\t" + obj.key());
		}
		logger.debug("\n");
		
		
//		  towards(agent1 box1)   011111111110
//		  forward(agent1)        111111000000
//		  down(speed(agent1))    000000111110
//		  towards(agent1 wall1)  011111111110
//		  collision(agent1 box1) 000000000001
		  
		set = new ArrayList<Interval>();
		set.add(Interval.make("distance-decreasing(agent,box)", 1, 10));
		set.add(Interval.make("forward(agent)", 0, 5));
		set.add(Interval.make("speed-decreasing(agent)", 5, 10));
		set.add(Interval.make("distance-decreasing(agent,box2)", 1, 10));
		set.add(Interval.make("collision(agent,box)", 10, 11));
		set.add(Interval.make("distance-stable(agent,box2)", 0, 0));
		set.add(Interval.make("distance-increasing(agent,box2)", 0, 0));
		set.add(Interval.make("turn-left(agent)", 0, 0));
		set.add(Interval.make("turn-right(agent)", 21, 21));
		Paint.render(set, "/Users/wkerr/Desktop/approach-ts-3.png");
		Collections.sort(set, Interval.esf);
		sequence = SequenceType.allen.getSequence(set);
		logger.debug("Sequence Size: " + sequence.size());	
		for (WeightedObject obj : sequence) { 
			logger.debug("\t" + obj.key());
		}
		logger.debug("\n");
		
//		  towards(agent1 box1)  01111111111
//		  forward(agent1)       11111100000
//		  down(speed(agent1))   00000011111
//		  turn-left(agent1)     00000111000
//		  turn-right(agent1)    00011000000
//		  towards(agent1 box2)  01110000000
//		  away(agent1 box2)     00000011111
		  		  
		set = new ArrayList<Interval>();
//		set.add(Interval.make("distance-decreasing(agent,box)", 1, 20));
//		set.add(Interval.make("forward(agent)", 0, 15));
//		set.add(Interval.make("speed-decreasing(agent)", 15, 20));
//		set.add(Interval.make("turn-left(agent)", 9, 12));
//		set.add(Interval.make("turn-right(agent)", 5, 9));
//		set.add(Interval.make("turn-right(agent)", 12, 15));
//		set.add(Interval.make("distance-decreasing(agent,box2)", 1, 8));
//		set.add(Interval.make("distance-increasing(agent,box2)", 13, 20));

		set.add(Interval.make("distance-decreasing(agent,box)", 1, 20));
		set.add(Interval.make("forward(agent)", 0, 15));
		set.add(Interval.make("speed-decreasing(agent)", 15, 20));
		set.add(Interval.make("turn-left(agent)", 9, 12));
		set.add(Interval.make("turn-right(agent)", 5, 9));
		set.add(Interval.make("turn-right(agent)", 12, 15));
		set.add(Interval.make("distance-decreasing(agent,box2)", 1, 8));
		set.add(Interval.make("distance-increasing(agent,box2)", 13, 20));
		set.add(Interval.make("distance-stable(agent,box2)", 0, 0));
		set.add(Interval.make("collision(agent,box)", 21, 21));
		Paint.render(set, "/Users/wkerr/Desktop/approach-ts-4.png");
		Collections.sort(set, Interval.esf);
		sequence = SequenceType.allen.getSequence(set);
		logger.debug("Sequence Size: " + sequence.size());	
		for (WeightedObject obj : sequence) { 
			logger.debug("\t" + obj.key());
		}
		logger.debug("\n");

		set = new ArrayList<Interval>();
		set.add(Interval.make("distance-decreasing(agent,box)", 1, 5));
		set.add(Interval.make("distance-decreasing(agent,box)", 11, 18));
		set.add(Interval.make("forward(agent)", 0, 14));
		set.add(Interval.make("speed-decreasing(agent)", 14, 18));
		set.add(Interval.make("turn-right(agent)", 5, 7));
		set.add(Interval.make("turn-left(agent)", 10, 13));
		set.add(Interval.make("distance-decreasing(agent,box2)", 1, 6));
		set.add(Interval.make("distance-increasing(agent,box2)", 14, 18));
		set.add(Interval.make("distance-stable(agent,box2)", 6, 14));
		set.add(Interval.make("collision(agent,box)", 21, 21));
		Paint.render(set, "/Users/wkerr/Desktop/approach-ts-5.png");
		Collections.sort(set, Interval.esf);
		sequence = SequenceType.allen.getSequence(set);
		logger.debug("Sequence Size: " + sequence.size());	
		for (WeightedObject obj : sequence) { 
			logger.debug("\t" + obj.key());
		}
		logger.debug("\n");	
	}
}
