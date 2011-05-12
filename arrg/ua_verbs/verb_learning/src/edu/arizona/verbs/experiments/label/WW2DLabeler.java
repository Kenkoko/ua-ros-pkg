package edu.arizona.verbs.experiments.label;

import java.util.List;

import edu.arizona.verbs.shared.OOMDPState;

public enum WW2DLabeler implements Labeler {
	go {
		@Override
		public Label label(List<OOMDPState> trace) {
			boolean goalReached = false;
			
			for (OOMDPState state : trace) {
				boolean atGoal = state.getActiveRelations().contains("Collision(person,place)");
				
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
	
	intercept {
		@Override
		public Label label(List<OOMDPState> trace) {
			for (OOMDPState state : trace) {
				boolean metEnemy = state.getActiveRelations().contains("Collision(person,enemy)");
				boolean enemyAtGoal = state.getActiveRelations().contains("Collision(enemy,place)");
				
				if (enemyAtGoal) {
					return Label.Violation;
				} else if (metEnemy) {
					return Label.Success;
				}
			}
			
			return Label.Neither;
		}
	},
	
	avoid {
		@Override
		public Label label(List<OOMDPState> trace) {
			for (OOMDPState state : trace) {
				boolean caught = (state.getActiveRelations().contains("Collision(person,enemy)") ||
								  state.getActiveRelations().contains("Collision(person,waypoint)"));
				boolean atGoal = state.getActiveRelations().contains("Collision(person,place)");
				
				if (caught) {
					return Label.Violation;
				} else if (atGoal) {
					return Label.Success;
				}
			}
			
			return Label.Neither;
		}
	},
	
	go_via {
		@Override
		public Label label(List<OOMDPState> trace) {
			int phase = 0; // start
			
			for (OOMDPState state : trace) {
				boolean atWaypoint = state.getActiveRelations().contains("Collision(person,waypoint)");
				boolean atGoal = state.getActiveRelations().contains("Collision(person,place)");
				
				switch (phase) {
				case 0:
					if (atWaypoint && atGoal) {
						phase = 2;
					} else if (!atWaypoint && atGoal) {
						return Label.Violation;
					} else if (atWaypoint){ // i.e., atWaypoint but not atGoal
						phase = 1;
					}
					break;
				case 1:
					if (atGoal) {
						phase = 2;
					}
					break;
				case 2:
					if (!atGoal) {
						return Label.Violation;
					}
					break;
				}
			}
			
			if (phase == 2) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	},
	
	meet {
		@Override
		public Label label(List<OOMDPState> trace) {
			int phase = 0; // start
			
			for (OOMDPState state : trace) {
				boolean otherAtGoal = state.getActiveRelations().contains("Collision(enemy,place)");
				boolean atGoal = state.getActiveRelations().contains("Collision(person,place)");
				
				switch (phase) {
				case 0: // Neither at the goal
					if (atGoal && otherAtGoal) {
						phase = 3;
					} else if (otherAtGoal) {
						phase = 2;
					} else if (atGoal) {
						phase = 1;
					}
					break;
				case 1: // Only you at the goal
					if (!atGoal) {
						return Label.Violation;
					} else if (otherAtGoal) {
						phase = 3;
					}
					break;
				case 2: // Only other at the goal
					if (atGoal) {
						phase = 3;
					}
					break;
				case 3: // Both at goal
					if (!atGoal) {
						return Label.Violation;
					}
					break;
				}
			}
			
			if (phase == 3) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	},
	
	follow {
		@Override
		public Label label(List<OOMDPState> trace) {
			int phase = 0; // start
			
			for (OOMDPState state : trace) {
				boolean otherAtGoal = state.getActiveRelations().contains("Collision(enemy,place)");
				boolean atGoal = state.getActiveRelations().contains("Collision(person,place)");
				
				switch (phase) {
				case 0: // Neither at the goal
					if (atGoal) {
						return Label.Violation;
					} else if (otherAtGoal) {
						phase = 1;
					} 
					break;
				case 1: // Other has reached goal
					if (atGoal) {
						phase = 2;
					}
					break;
				case 2: // Both at goal
					if (!atGoal) {
						return Label.Violation;
					}
					break;
				}
			}
			
			if (phase == 2) {
				return Label.Success;
			} else {
				return Label.Neither;
			}
		}
	}
	
	;
	
	public abstract Label label(List<OOMDPState> trace);
}
