package edu.arizona.cs.learn.timeseries.visualization;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashSet;
import java.util.Properties;
import java.util.Set;

import edu.arizona.cs.learn.timeseries.visualization.graphics.DataComponent;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

public abstract class Controller {

	protected String fileName = ".msa.properties";
	protected Properties _properties;
	protected DataModel _dataModel;
	
	protected Set<DataComponent> _listeners;

	public Controller() {
		super();
		
		_listeners = new HashSet<DataComponent>();
	}

	/**
	 * Load the properties into our properties object.
	 */
	protected void loadProperties() {
	    FileInputStream fin = null;
		_properties = new Properties();
	    try {
	        fin = new FileInputStream(fileName);
	        _properties.load(fin);
	        fin.close();
	    } catch (Exception e) {
	    	System.out.println("Properties don't exist, will create after first save");
	    }
	}

	/**
	 * Save the properties with the new path.
	 * @param newPath
	 */
	public void saveProps() {
        FileOutputStream fout;
        try {
            fout = new FileOutputStream(fileName);
            _properties.store(fout, "Properties");
            fout.close();
        } catch (IOException e) {
            System.out.println("Could not save properties: " + e.toString());
        }
	}	
	
	
	public void saveDirectory(String name, String path) {
		int fileStart = path.lastIndexOf(File.separator);
		String directory = path .substring(0, fileStart);
		_properties.setProperty(name, directory);
		saveProps();
	}

	/**
	 * This sets the current DataModel to a new model created
	 * @param model
	 */
	public abstract void setModel(DataModel model);


	public void sendMessage(String message) { 
		for (DataComponent dc : _listeners) { 
			dc.receiveMessage(message);
		}
	}

	public void add(DataComponent component) {
		_listeners.add(component);
	}
}