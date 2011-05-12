package edu.arizona.verbs.experiments.label;

import java.util.List;

import edu.arizona.verbs.shared.OOMDPState;

public enum GazeboLabeler implements Labeler {
	go {
		@Override
		public Label label(List<OOMDPState> trace) {
			boolean goalReached = false;
			
			for (OOMDPState state : trace) {
				boolean atGoal = state.getActiveRelations().contains("Contact(robot_description,goal)");
				
				if (atGoal) {
					goalReached = true;
				} else if (goalReached) {
					return Label.Violation; // Entering the goal and then leaving it is a violation
				}
			}
			
			if (goalReached) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	},
	
	deliver {
		@Override
		public Label label(List<OOMDPState> trace) {
			DeliverStages phase = DeliverStages.Approaching;
			
			for (OOMDPState state : trace) {
				boolean atItem = state.getActiveRelations().contains("Contact(robot_description,item)");
				boolean atGoal = state.getActiveRelations().contains("Contact(robot_description,goal)");
				boolean holding = state.getActiveRelations().contains("Carrying(robot_description,item)"); 

//				System.out.println(state.getActiveRelations());
//				System.out.println(atItem + ", " + atGoal + ", " + holding + ": " + phase);
				
				switch (phase) {
				case Approaching: // Not holding
					if (holding) {
						phase = DeliverStages.Carrying;
					} else if (atItem) {
						phase = DeliverStages.AtItem;
					}
					break;
				case AtItem: // At item
					if (holding) {
						phase = DeliverStages.Carrying;
					} else if (!atItem) {
						return Label.Violation; // Left without picking it up
					}
					break;
				case Carrying: // Holding item
					if (!holding) {
						if (atGoal) {
							phase = DeliverStages.Delivered;
						} else {
							return Label.Violation; // dropped before reaching goal
						}
					}
					break;
				case Delivered: // At goal, object delivered
					if (!atGoal) {
						return Label.Violation;  // Left goal
					}
					break;
				}
//				System.out.println("AFTER: " + phase);
			}
			
			if (phase == DeliverStages.Delivered) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	}
	
	;
	
	enum DeliverStages {Approaching, AtItem, Carrying, Delivered}; // No local enums, annoying
	
	public abstract Label label(List<OOMDPState> trace);
}
