package edu.arizona.cs.learn.timeseries.prep.ww2d;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.sql.ResultSet;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;


public class ExportStreamFile {
	
	public static String[] classes = new String[] { 
//		"eat", "chase", "fight", "flee", "kick-ball", "kick-column"
//		"collide", "pass", "talk-a", "talk-b"
		"fight"
	};
	
	public static String inPrefix = "data/raw-data/ww2d/states/";
	public static String outPrefix = "data/raw-data/ww2d/";
	public static int numEpisodes = 100;
	
	/**
	 * Change this function when you want to change which
	 * file is output and where it is output to.
	 * @param args
	 */
	public static void main(String[] args) { 
//		convertActivities();

		convert(false);
		convert(true);
	}
	
	public static void convert(boolean agent) { 

		String out = outPrefix + "global/";
		String db = "/state-global.db";
		if (agent) {
			out = outPrefix + "agent/";
			db = "/state-agent1.db";
		} 
		
		for (String className : classes) { 
			
			// Ensure that the output directory exists.
			File f = new File(out + className + "/");
			if (!f.exists()) 
				f.mkdir();

			for (int i = 1; i <= 100; ++i) { 
				String inputFile = inPrefix + "/" + className + "-" + i + db;
				String outputFile = out + className + "/" + className + "-" + i + ".csv";
				convert(inputFile, outputFile);
			}
		}
	}

	
	public static void convert(String inputDb, String outputFile) { 
		Map<String,List<Row>> map = getRows(inputDb);
		Map<String,String[]> streamMap = toStreams(map);
		writeCSV(streamMap, outputFile);
	}
	
	
	
	
	public static void convertActivities() { 
		String path = "states/";
		
		int min = 1;
		int max = 20;
		String[] activity = { "ball", "chase", "column", "eat", "fight", "wander" };

		for (String base : activity) { 
			for (int cnt = min; cnt <= max; ++cnt) {

				String statePath = path + base + "-" + cnt + "/";
				String inputDB = statePath + "state-agent1.db";
				String outputFile = "logs/" + base + "-" + cnt + ".csv";

				Map<String,List<Row>> localMap = getRows(inputDB);
				Map<String,String[]> streamMap = toStreams(localMap);
				writeCSV(streamMap, outputFile);
			}				
		}		
	}
	
	/**
	 * Extract all of the rows from the SQLite database.  These are a fluent
	 * representation where each row corresponds to a value that is consistent
	 * during some interval.
	 * @param inputDb
	 * @return
	 */
	public static Map<String,List<Row>> getRows(String inputDb) { 
		StateDatabase db = new StateDatabase(inputDb, false);
		Map<Integer,String> entityMap = new HashMap<Integer,String>();
		Map<Integer,String> fluentMap = new HashMap<Integer,String>();

		fillMap(db, "select rowid, name from lookup_fluent_table", fluentMap);
		fillMap(db, "select rowid, name from entity_map_table", entityMap);

		System.out.println("Fluents: " + fluentMap.size() + " Entities: " + entityMap.size());

		Map<String,List<Row>> rowMap = new HashMap<String,List<Row>>();
		for (String fluent : fluentMap.values()) { 
			String tableName = "fluent_" + fluent;

			try { 
				Statement s = db.getStatement();
				ResultSet rs = s.executeQuery("select * from " + tableName);
				while (rs.next()) { 
					Row r = new Row();
					r.fluent = fluent;
					r.entities = entityMap.get(rs.getInt("entities_id")).replaceAll("[,]", "");
					r.value = rs.getString("value");
					r.start = rs.getInt("start_time");
					r.end = rs.getInt("end_time");

					// we are going to try and keep track of the exact set of 
					// bounded fluents.
					String key = "\"" + fluent + "(" + r.entities + ")\"";
					List<Row> rows = rowMap.get(key);
					if (rows == null) { 
						rows = new ArrayList<Row>();
						rowMap.put(key, rows);
					}
					rows.add(r);
				}
				rs.close();
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
		
		return rowMap;
	}
	
	/**
	 * Covert the fluent representation into a stream representation since that
	 * is what the CSV will be expecting.
	 * @param rowMap
	 * @return
	 */
	public static Map<String,String[]> toStreams(Map<String,List<Row>> rowMap) { 
		// First we have to determine the max time that the fluents last....
		int maxTime = 0;  // this will contain the size of the stream to be created.
		for (List<Row> rows : rowMap.values()) { 
			for (Row r : rows) 
				maxTime = Math.max(r.end, maxTime);
		}

		// now we need to convert each fluent/entities pair into a stream
		Map<String,String[]> streamMap = new TreeMap<String,String[]>();
		for (String key : rowMap.keySet()) { 
			for (Row r : rowMap.get(key)) { 
				// first check to see if we already created a stream for this entity.  If not add it.
				String[] stream = streamMap.get(key);
				if (stream == null) { 
					stream = new String[maxTime];
					streamMap.put(key, stream);
				}

				for (int i = r.start; i < r.end; ++i) { 
					// if you want numerical values instead of "true" and "false" and "unknown"
					// then uncomment out this line.  Otherwise, leave it be.
					//					updateNumeric(stream, r.value, i);
					stream[i] = r.value;
				}
			}
		}
		return streamMap;
	}
	
	/**
	 * Write the CSV file from the streams given.
	 * @param streamMap
	 * @param outputFile
	 */
	public static void writeCSV(Map<String,String[]> streamMap, String outputFile) { 
		// now we can write the mofo out.
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter(outputFile));
			StringBuffer header = new StringBuffer();
			
			// write out the header
			int maxTime = 0;
			for (String name : streamMap.keySet()) { 
				header.append(name + ",");
				maxTime = Math.max(maxTime, streamMap.get(name).length);
			}	
			header.deleteCharAt(header.length()-1);
			out.write(header + "\n");

			// time starts at 1 so we should too.
			for (int i = 1; i < maxTime; ++i) { 
				StringBuffer row = new StringBuffer();
				for (Map.Entry<String,String[]> entry : streamMap.entrySet()) { 
					String value = entry.getValue()[i];
					row.append(value + ",");
				}
				row.deleteCharAt(row.length()-1);
				out.write(row + "\n");
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}

	}
	
	/**
	 * Update the stream at position i with the value 
	 * @param stream
	 * @param r
	 * @param i
	 */
	private static void updateNumeric(String[] stream, String value, int i) { 
		if ("false".equals(value)) {
			stream[i] = "0";
		} else if ("true".equals(value)) { 
			stream[i] = "1";
		} else if ("unknown".equals(value)){
			stream[i] = "-1";
		} else { 
			stream[i] = value;
		}
	}
	
	/**
	 * Populate a map of integers mapped to the name of the thing.
	 * @param db
	 * @param sql
	 * @param map
	 */
	private static void fillMap(StateDatabase db, String sql, Map<Integer,String> map) { 
		try { 
			Statement s = db.getStatement();
			ResultSet rs = s.executeQuery(sql);
			while (rs.next()) {
				int id = rs.getInt("rowid");
				String name = rs.getString("name");
				map.put(id, name);
			}
			rs.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
}

class Row { 
	public String fluent;
	public String entities;
	
	public String value;
	public int start;
	public int end;
}