package edu.arizona.teach;

import java.util.ArrayList;
import java.util.HashMap;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;

/**
 *
 * @author diana
 */

public class ScenarioBank {
    static HashMap<String, ArrayList<MDPObjectState>> scenarios;

        //empty constructor
public ScenarioBank(){}

/* Add new scenario to Scenario Bank */
public void addScene(String newName, ArrayList<MDPObjectState> newItems) {

    if  (scenarios == null)
        scenarios = new HashMap<String,ArrayList<MDPObjectState>>();

    if (scenarios.containsKey(newName))
        System.out.println("Scenario " + newName + " already exists.");
    else
        scenarios.put(newName, newItems);
  }

/* Get list of MDPObjectStates that are in selected scenario */
public ArrayList<MDPObjectState> getScene(String name) {
    if (scenarios == null || !scenarios.containsKey(name)) {
        System.out.println("No scenario named " + name + " exists.");
        return null;
    }
    else
        return scenarios.get(name);
}

/* Delete selected scenario from Scenario Bank */
public void deleteScene(String name) {
    if (scenarios == null || !scenarios.containsKey(name)) {
        System.out.println("You can't delete " + name + " because it doesn't exist.");
    }
    else
        scenarios.remove(name);
}

/* Get a list of scenarios present in Scenario Bank */
public String[] seeScenes() {
    if (scenarios == null) {
        System.out.println("No scenarios exist.");
        return null;
    }
    else {
        String[] names = new String[scenarios.size()];
        Object[] keys = scenarios.keySet().toArray();
        for (int i = 0; i < scenarios.size(); i++)
            names[i] = (String)keys[i];
        return names;
    }
    }
}