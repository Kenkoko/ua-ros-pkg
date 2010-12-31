package edu.arizona.cs.learn.timeseries.model;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.symbols.CBA;
import edu.arizona.cs.learn.timeseries.model.symbols.Event;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public enum SequenceType {
	starts {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			Collections.sort(intervals, Interval.starts);
			return propSequence(intervals);
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			return intervals.size();
		}

	}, 
	ends {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			Collections.sort(intervals, Interval.ends);
			return propSequence(intervals);
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			return intervals.size();
		}
	}, 
	startsEnds {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			// first thing is to make a timeline
			int maxTime = -1;
			for (Interval interval : intervals) { 
				maxTime = Math.max(maxTime, interval.end);
			}
			
			List<Symbol> sequence = new ArrayList<Symbol>();
			for (int i = 0; i <= maxTime; ++i) {
				List<Interval> events = new ArrayList<Interval>();
				for (Interval interval : intervals) { 
					if (interval.start == i || interval.end == i) 
						events.add(interval);
				}
				
				Collections.sort(events, new Comparator<Interval>() {
					@Override
					public int compare(Interval o1, Interval o2) {
						return o1.name.compareTo(o2.name);
					} 
				});
				
				for (Interval interval : events) { 
					if (interval.start == i ) 
						sequence.add(new Event(interval.name+"_on", interval));
					if (interval.end == i) 
						sequence.add(new Event(interval.name+"_off", interval));
				}
			}
			return sequence;
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			return 2*intervals.size();
		}
	}, 
	allen {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			Collections.sort(intervals, Interval.eff);
			List<Symbol> results = new ArrayList<Symbol>();
			
			class Data { 
				public Symbol obj;
				public Interval i1;
				public Interval i2;
				
				public int start;
				public int end;
				
				public Data(Symbol obj, Interval i1, Interval i2, int start, int end) { 
					this.obj = obj;
					this.i1 = i1;
					this.i2 = i2;
					this.start = start;
					this.end = end;
				}
			}
			
			List<Data> tmp = new ArrayList<Data>();
			for (int i = 0; i < intervals.size(); ++i) { 
				Interval i1 = intervals.get(i);
				for (int j = i+1; j < intervals.size(); ++j) { 
					Interval i2 = intervals.get(j);
					
					if (!Utils.LIMIT_RELATIONS || i2.start - i1.end < Utils.WINDOW) { // or 5 for most things....
						String relation = AllenRelation.get(i1, i2);
						AllenRelation allen = new AllenRelation(relation, i1, i2);
						
						tmp.add(new Data(allen, i1, i2, Math.min(i1.start,i2.start), Math.max(i1.end, i2.end)));
					}
				}
			}
			
			Collections.sort(tmp, new Comparator<Data>() {
				@Override
				public int compare(Data o1, Data o2) {
					if (o1.end > o2.end) 
						return 1;
					if (o1.end < o2.end)
						return -1;

					if (o1.start > o2.start)
						return 1;
					if (o1.start < o2.start) 
						return -1;

					if (o1.i1.end > o2.i1.end)
						return 1;
					if (o1.i1.end < o2.i1.end)
						return -1;
					
					if (o1.i2.start > o2.i2.start)
						return 1;
					if (o1.i2.start < o2.i2.start)
						return -1;
					
					int name1 = o1.i1.name.compareTo(o2.i1.name);
					if (name1 > 0 || name1 < 0)
						return name1;

					int name2 = o1.i2.name.compareTo(o2.i2.name);
					if (name2 > 0 || name2 < 0)
						return name2;

					throw new RuntimeException("Two relations: " + o1.obj + " " + o2.obj);
				} 
			});
			
			for (Data d : tmp)
				results.add(d.obj);
			return results;
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			int count = 0;
			for (int i = 0; i < intervals.size(); i++) {
				Interval i1 = intervals.get(i);
				for (int j = i + 1; j < intervals.size(); j++) {
					Interval i2 = (Interval) intervals.get(j);

					if ((!Utils.LIMIT_RELATIONS) || (i2.start - i1.end < Utils.WINDOW)) {
						count++;
					}
				}
			}
			return count;
		}
	}, 
	randomStarts {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			List<Interval> shortList = new ArrayList<Interval>();
			for (Interval orig : intervals) { 
				int randomStart = Utils.random.nextInt(orig.end-orig.start);
				shortList.add(Interval.make(orig.name, orig.start + randomStart, orig.start + randomStart+1));
			}
			
			Collections.sort(shortList, Interval.starts);
			return propSequence(shortList);
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			return intervals.size();
		}
	}, 
	bpp {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			List<Symbol> results = new ArrayList<Symbol>();
//			logger.debug("pre: " + list);
			
			// sort them by earliest start time... and who cares about the finish time
			List<CBA> tmp = new ArrayList<CBA>();
			Collections.sort(intervals, Interval.eff);
			for (int i = 0; i < intervals.size(); ++i) { 
				Interval i1 = intervals.get(i);
				for (int j = i+1; j < intervals.size(); ++j) { 
					Interval i2 = intervals.get(j);
					for (int k = j+1; k < intervals.size(); ++k) { 
						Interval i3 = intervals.get(k);

						if (!Utils.LIMIT_RELATIONS || Interval.interact(i1, i2, i3, Utils.WINDOW)) {
							CBA cba = new CBA();
							cba.addInterval(i1);
							cba.addInterval(i2);
							cba.addInterval(i3);
							cba.finish();
							tmp.add(cba);
						} else { 
//							logger.debug("Intervals don't interact");
//							logger.debug("\n\t" + i1.toString() + "\n\t" + i2.toString() + "\n\t" + i3.toString());
						}
					}
				}
			}
			
			Collections.sort(tmp);
			results.addAll(tmp);
			
			// print the sequence....
//			logger.debug("**** SEQUENCE **** ");
//			for (CBA cba : tmp) {
//				logger.debug("\t\t" + cba.toString());
//			}
			return results;
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			int count = 0;
			Collections.sort(intervals, Interval.eff);
			for (int i = 0; i < intervals.size(); i++) {
				Interval i1 = intervals.get(i);
				for (int j = i + 1; j < intervals.size(); j++) {
					Interval i2 = intervals.get(j);
					for (int k = j + 1; k < intervals.size(); k++) {
						Interval i3 = intervals.get(k);

						if ((!Utils.LIMIT_RELATIONS) || (Interval.interact(i1, i2, i3, Utils.WINDOW))) {
							count++;
						}
					}
				}
			}
			return count;
		}
	},
	fullCBA {
		@Override
		public List<Symbol> getSequence(List<Interval> intervals) {
			List<Interval> list = BPPFactory.compress(intervals, Interval.eff);
			int maxTime = 0;
			for (Interval i : list) 
				maxTime = Math.max(maxTime, i.end);
				
			for (int i = 0; i < maxTime; ++i) { 
				
			}
			// TODO Auto-generated method stub
			return null;
		}

		@Override
		public int getSequenceSize(List<Interval> intervals) {
			// TODO Auto-generated method stub
			return 0;
		} 
		
	};

	public abstract List<Symbol> getSequence(List<Interval> intervals);
	public abstract int getSequenceSize(List<Interval> intervals);

	public static List<SequenceType> get(String option) {
		List<SequenceType> list = new ArrayList<SequenceType>();

		if ("all".equals(option)) {
			list.add(starts);
			list.add(ends);
			list.add(startsEnds);
			list.add(allen);
		} else if ("noRelations".equals(option)) {
			list.add(starts);
			list.add(ends);
			list.add(startsEnds);
		} else {
			list.add(valueOf(option));
		}
		return list;
	}
	
	/**
	 * Output a sequence of the names of the proposition (the list
	 * should already be sorted the way you want it to be.
	 * @param list
	 * @return
	 */
	public static List<Symbol> propSequence(List<Interval> list) { 
		List<Symbol> results = new ArrayList<Symbol>();
		for (int i = 0; i < list.size(); ++i) { 
			Interval i1 = list.get(i);
			results.add(new Event(i1));
		}
		return results;
		
	}
}
