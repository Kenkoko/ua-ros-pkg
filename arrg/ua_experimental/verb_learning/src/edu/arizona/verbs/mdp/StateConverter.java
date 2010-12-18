package edu.arizona.verbs.mdp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.msg.MDPState;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;

import edu.arizona.cs.learn.algorithm.alignment.factory.SequenceFactory;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Relation;

public class StateConverter {
	
	// TODO: Clean up this class now that rosjava is officially released
	
	/* Arrays */
	
	public static ArrayList<MDPObjectState> objectsToMsgArrayList(List<OOMDPObjectState> objectList) {
		return new ArrayList<MDPObjectState>(
				Collections2.transform(objectList, new Function<OOMDPObjectState, MDPObjectState>() {
					public MDPObjectState apply(OOMDPObjectState state) { return objStateToMsg(state); }}));
	}
	
	public static List<OOMDPState> msgArrayToStates(ArrayList<MDPState> trace) {
		List<OOMDPState> result = new Vector<OOMDPState>();
		for (int i = 0; i < trace.size(); i++) {
			result.add(msgToState(trace.get(i)));
		}
		
		return result;
	}
	
	public static ArrayList<MDPState> stateToMsgArrayList(List<OOMDPState> trace) {
		ArrayList<MDPState> mdpStates = new ArrayList<MDPState>();
		for (OOMDPState s : trace) {
			mdpStates.add(stateToMsg(s));
		}
		
		return mdpStates;
	}
	
	
	/* OOMDPObjectState */ 
	
	public static OOMDPObjectState msgToObjState(MDPObjectState message) {
		OOMDPObjectState state = new OOMDPObjectState(message.name, message.class_name);
		state.setAttributes(message.attributes, message.values);
		return state;
	}
	
	public static MDPObjectState objStateToMsg(OOMDPObjectState state) {
		MDPObjectState message = new MDPObjectState();
		
		message.class_name = state.getClassName();
		message.name = state.getName();
		message.attributes = state.getAttributes();
		message.values = state.getValues();
		
		return message;
	}
	
	/* OOMDPState */
	
	public static OOMDPState msgToState(MDPState stateMessage) {
		Vector<OOMDPObjectState> objectStates = new Vector<OOMDPObjectState>();
	
		
		for (MDPObjectState objStateMsg : stateMessage.object_states) {
			objectStates.add(msgToObjState(objStateMsg));
		}
		
		Vector<Relation> relations = new Vector<Relation>();
		for (ros.pkg.oomdp_msgs.msg.Relation r : stateMessage.relations) {
			relations.add(convertRelation(r));
		}
		
		return new OOMDPState(objectStates, relations);
	}
	
	public static MDPState stateToMsg(OOMDPState state) {
		MDPState message = new MDPState();
		
		message.object_states = new ArrayList<MDPObjectState>(
				Collections2.transform(state.getObjectStates(), new Function<OOMDPObjectState, MDPObjectState>() {
					public MDPObjectState apply(OOMDPObjectState state) { return objStateToMsg(state); }
				}));
		
		for (Relation r : state.getRelations()) {
			message.relations.add(deconvertRelation(r));
		}
		
		return message;
	}
	
	/* Relation */ 
	
	public static Relation convertRelation(ros.pkg.oomdp_msgs.msg.Relation relMsg) {
		return new Relation(relMsg.relation, relMsg.obj_names, relMsg.value);
	}
	
	public static ros.pkg.oomdp_msgs.msg.Relation deconvertRelation(Relation relation) {
		ros.pkg.oomdp_msgs.msg.Relation relMsg = new ros.pkg.oomdp_msgs.msg.Relation();
		relMsg.relation = relation.relation;
		relMsg.obj_names = relation.objectNames;
		relMsg.value = relation.value;
		return relMsg;
	}
	
	/* Traces (Lists of States) */
	
	public static Instance convertTrace(List<OOMDPState> trace, String verb, Map<String, String> nameMap) {
		ArrayList<Interval> closedIntervals = new ArrayList<Interval>();
		Map<String, Interval> openIntervals = new HashMap<String, Interval>();

		for (int i = 0; i < trace.size(); i++) {
			OOMDPState state = trace.get(i);
			List<Relation> relations = state.getRelations();
			for (Relation relation : relations) {
				Relation remapped = relation.remap(nameMap);
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
