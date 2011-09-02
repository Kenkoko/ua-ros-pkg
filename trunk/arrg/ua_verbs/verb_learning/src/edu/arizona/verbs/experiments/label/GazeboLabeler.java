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
	},
	
	go_via {
		@Override
		public Label label(List<OOMDPState> trace) {
			GoViaStages phase = GoViaStages.PreWaypoint;
			
			for (OOMDPState state : trace) {
				boolean atWaypoint = state.getActiveRelations().contains("Contact(robot_description,waypoint)");
				boolean atGoal = state.getActiveRelations().contains("Contact(robot_description,goal)");
				
//				System.out.println(atWaypoint + ", " + atGoal + ": " + phase);
//				System.out.println(state.getActiveRelations());
				
				switch (phase) {
				case PreWaypoint: 
					if (atWaypoint) {
						phase = GoViaStages.AtWaypoint;
					} else if (atGoal) {
						return Label.Violation; // Went to goal first
					}
					break;
				case AtWaypoint:
					if (atGoal) {
						phase = GoViaStages.AtGoal;
					} else if (!atWaypoint) {
						phase = GoViaStages.PreGoal;
					}
					break;
				case PreGoal: 
					if (atGoal) {
						phase = GoViaStages.AtGoal;
					} else if (atWaypoint) {
						return Label.Violation; // Went back to waypoint
					}
					break;
				case AtGoal: 
					if (!atGoal) {
						return Label.Violation;  // Left goal
					}
					break;
				}
			}
			
//			System.out.println("AFTER: " + phase);
			
			if (phase == GoViaStages.AtGoal) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	},
	
	remove {
		@Override
		public Label label(List<OOMDPState> trace) {
			RemoveStages phase = RemoveStages.PreLocation;
			
			for (OOMDPState state : trace) {
				boolean atLoc = state.getActiveRelations().contains("Contact(robot_description,goal)");
				boolean holding = state.getActiveRelations().contains("Carrying(robot_description,item)"); 

				System.out.println(state.getActiveRelations());
				System.out.println(atLoc + ", " + holding + ": " + phase);
				
				switch (phase) {
				case PreLocation: 
					if (atLoc) {
						phase = RemoveStages.AtLocation;
					}
					break;
				case AtLocation: 
					if (holding) {
						phase = RemoveStages.HoldingAtLocation;
					} else if (!atLoc) {
						return Label.Violation; // Left without picking it up
					}
					break;
				case HoldingAtLocation:
					if (!holding) {
						return Label.Violation; // dropped before leaving goal
					} else if (!atLoc) {
						phase = RemoveStages.HoldingOffLocation;
					}
					break;
				case HoldingOffLocation: 
					if (atLoc) {
						return Label.Violation;  // went back to goal
					} else if (!holding) {
						phase = RemoveStages.Dropped;
					}
					break;
				case Dropped:
					if (holding) {
						return Label.Violation;  // Picked it up again
					}
				}
				System.out.println("AFTER: " + phase);
			}
			
			if (phase == RemoveStages.Dropped) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	}
	
	;
	
	enum DeliverStages {Approaching, AtItem, Carrying, Delivered}; // No local enums, annoying
	enum GoViaStages {PreWaypoint, AtWaypoint, PreGoal, AtGoal};
	enum RemoveStages {PreLocation, AtLocation, HoldingAtLocation, HoldingOffLocation, Dropped};
	
	public abstract Label label(List<OOMDPState> trace);
}
