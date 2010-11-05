package edu.arizona.verbs.mdp;

import java.util.List;
import java.util.Vector;

import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.msg.MDPState;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;

public class StateConverter {
	
	/* Arrays */
	
	public static MDPObjectState[] objectsToMsgArray(List<OOMDPObjectState> objectList) {
		return Collections2.transform(objectList, new Function<OOMDPObjectState, MDPObjectState>() {
					public MDPObjectState apply(OOMDPObjectState state) { return objStateToMsg(state); }})
			  .toArray(new MDPObjectState[0]);
	}
	
	public static List<OOMDPState> msgArrayToStates(MDPState[] trace) {
		List<OOMDPState> result = new Vector<OOMDPState>();
		
		for (int i = 0; i < trace.length; i++) {
			result.add(msgToState(trace[i]));
		}
		
		return result;
	}
	
	public static MDPState[] stateToMsgArray(List<OOMDPState> trace) {
		MDPState[] mdpStates = new MDPState[trace.size()];

		for (int i = 0; i < trace.size(); i++) {
			mdpStates[i] = stateToMsg(trace.get(i));
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
		message.attributes = state.getAttributes().toArray(new String[0]);
		message.values = state.getValues().toArray(new String[0]);
		
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
		
		message.object_states = Collections2.transform(state.getObjectStates(), new Function<OOMDPObjectState, MDPObjectState>() {
			public MDPObjectState apply(OOMDPObjectState state) { return objStateToMsg(state); }
		}).toArray(new MDPObjectState[0]);
		
		message.relations = new ros.pkg.oomdp_msgs.msg.Relation[state.getRelations().size()];
		for (int i = 0; i < state.getRelations().size(); i++) {
			message.relations[i] = deconvertRelation(state.getRelations().get(i)); 
		}
		
		return message;
	}
	
	public static Relation convertRelation(ros.pkg.oomdp_msgs.msg.Relation relMsg) {
		return new Relation(relMsg.relation, relMsg.obj_names, (relMsg.value > 0 ? true : false));
	}
	
	public static ros.pkg.oomdp_msgs.msg.Relation deconvertRelation(Relation relation) {
		ros.pkg.oomdp_msgs.msg.Relation relMsg = new ros.pkg.oomdp_msgs.msg.Relation();
		relMsg.relation = relation.relation;
		relMsg.obj_names = relation.getObjectNameArray();
		relMsg.value = (relation.value ? (byte) 1 : (byte) 0);
		return relMsg;
	}
}
