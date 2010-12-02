package edu.arizona.cs.learn.timeseries.prep.ww2d;

import java.io.File;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

public class StateDatabase implements StateDatabaseI {
    private static Logger logger = Logger.getLogger( StateDatabase.class );
    
    public static String PATH = "states/";

    protected Map<String,Integer> _fluentIdMap;
	protected Map<String,Integer> _entityKeyMap;
	protected Map<String,Integer> _rowIdMap;
	
	protected Map<String,PreparedStatement> _insertMap;
	protected Map<String,PreparedStatement> _updateMap;
	protected Map<String,PreparedStatement> _queryMap;
	
	protected Connection _conn;
	protected int _sessionId;
	
	protected Statement _defaultStatement;
	protected Statement _batchStatement;
	
	protected int _eventID = 0;
	
	protected PreparedStatement _paramsPS;
	protected PreparedStatement _eventPS;

	public StateDatabase(String name) {
		this(PATH + "state-" + name + ".db", true);
	}
	
	public StateDatabase(String file, boolean overwrite) { 
		_fluentIdMap = new HashMap<String,Integer>();
		_entityKeyMap = new HashMap<String,Integer>();
		_rowIdMap = new HashMap<String,Integer>();
		
		_insertMap = new HashMap<String,PreparedStatement>();
		_updateMap = new HashMap<String,PreparedStatement>();
		_queryMap = new HashMap<String,PreparedStatement>();

		connect(file, overwrite);
	}
	
	protected void connect(String file, boolean overwrite) {
		try {
			File f = new File(file);
			
			boolean initialize = false;
			if (overwrite && f.exists()) { 
				f.delete();
				initialize = true;
			} else if (!f.exists()) { 
				initialize = true;
			}
			
			Class.forName("org.sqlite.JDBC");
			_conn = DriverManager.getConnection("jdbc:sqlite:" + file);
			_conn.setAutoCommit(false);

			_defaultStatement = null;
			
			if (initialize) 
				initializeDatabase();
			else 
				loadExistingData();
			
			_eventPS = _conn.prepareStatement(INSERT_EVENT);
			_paramsPS = _conn.prepareStatement(INSERT_PARAMS);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public void disconnect() {
		try {
			_conn.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Initialize the database by adding all of the correct tables
	 * @throws Exception
	 */
	protected void initializeDatabase() throws Exception {
		Statement stat = getStatement();
		
		stat.executeUpdate(CREATE_ENTITY_LIST_TABLE);
		stat.executeUpdate(CREATE_FLUENT_LOOKUP_TABLE);
		stat.executeUpdate(CREATE_EVENT_TABLE);
		stat.executeUpdate(CREATE_PARAMS_TABLE);
		
		commit();
	}
	
	/**
	 * Load in the existing data so that we don't screw shit up.
	 * @throws Exception
	 */
	protected void loadExistingData() throws Exception { 
		// first we need to load in all of the already mapped fluent tables
		Statement stat = getStatement();
		ResultSet rs = stat.executeQuery("select rowid, name from lookup_fluent_table");
		while (rs.next()) { 
			int id = rs.getInt("rowid");
			String name = rs.getString("name");
			
			_fluentIdMap.put(name, id);
		}
		rs.close();
		
		rs = stat.executeQuery("select rowid, name from entity_map_table");
		while (rs.next()) { 
			int id = rs.getInt("rowid");
			String name = rs.getString("name");
			
			_entityKeyMap.put(name, id);
		}
		rs.close();
	}
	
	/**
	 * Returns a default statement that can used.  Reduces the
	 * number of statements that need to be created.
	 * @return
	 * @throws Exception
	 */
	public Statement getStatement() throws Exception {
		if (_defaultStatement == null)
			_defaultStatement = _conn.createStatement();
		return _defaultStatement;
	}
	
	/**
	 * Returns a newly created statement that can be used
	 * to execute SQL queries.
	 * @return
	 * @throws Exception
	 */
	public Statement createStatement() throws Exception {
		return _conn.createStatement();
	}

	/**
	 * Commit outstanding data to the database.  Use sparingly
	 * because it could take some time to finish the commit.
	 */
	public void commit() { 
		boolean notDone = true;
		
		int maxTries = 20;
		int numTries = 0;
		while (notDone && numTries < maxTries) {
			try {
				_conn.commit();
				notDone = false;
			} catch (Exception e) {
				logger.error("... commit failed: " + e.getMessage());
				++numTries;
				Thread.yield();
			}
		}
	}
	
	/**
	 * Get the next rowId prior to insertion for the fluentName
	 * given.  Each time this method is called, the stored value
	 * is incremented.  So calling it twice in a row will give you
	 * two different IDs
	 * @param fluentName
	 * @return
	 */
	public int getNextRowId(String fluentName) { 
		Object o = _rowIdMap.get(fluentName);
		if (o == null) {
			throw new RuntimeException("Unknown table: " + fluentName);
		}
		
		int i = (Integer) o;
		_rowIdMap.put(fluentName, i+1);
		return i;
	}

	/**
	 * Find or Add fluent will find the fluent id if it has already
	 * been added to the database.  If not, it will be added to the
	 * database and the newly assigned id will be returned.
	 * @param name
	 * @return
	 */
	public int findOrAddFluent(String name) { 
		Integer id = _fluentIdMap.get(name);
		if (id == null) { 
			id = createFluentTable(name);
		}
		return id;
	}
	
	/**
	 * dynamically allow the creation of new relation tables.
	 * this effectively lets us break up the tables into smaller
	 * workable chunks.
	 * 
	 * We may even allow the type of the table to become part
	 * of the table name.  For predicate based fluents this proves
	 * to be beneficial since predicates tend to last shorter amounts
	 * of time.
	 * @param tableName
	 */
	protected int createFluentTable(String tableName) {
		String table = CREATE_FLUENT_TABLE.replaceFirst("STUB", tableName);
//		String index = CREATE_FLUENT_INDEX_1.replaceAll("STUB", tableName);
		
		int id = -1;
		try {
			Statement s = getStatement();
			s.executeUpdate(table);
			s.executeUpdate("insert into lookup_fluent_table (name) values ('" + tableName + "')");
			
			id = getId("lookup_fluent_table", tableName);
			_fluentIdMap.put(tableName, id);
			_rowIdMap.put(tableName, 0);
//			s.executeUpdate(index);
		} catch (Exception e) {
			logger.error("Error creating table: " + table);
		} 
		return id;
	}
	
	/**
	 * @param name
	 * @return
	 */
	public int findOrAddEntityKey(String key) {
		Integer id = _entityKeyMap.get(key);
		if (id == null) { 
			id = addEntityKey(key);
		}
		return id;
	}
	
	/**
	 * add an entity to the entity table.
	 * @param name
	 * @param id
	 */
	private int addEntityKey(String key) {
		int id = -1;
		try {
			Statement s = getStatement();
			s.executeUpdate("insert into entity_map_table (name) " +
					"values ('" + key + "')");
			
			id = getId("entity_map_table", key);
			_entityKeyMap.put(key, id);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return id;
	}

	/**
	 * Assume that you've just inserted a row into tableName
	 * and now you need to get it back out.
	 * @param tableName
	 * @param name
	 * @return
	 */
	private int getId(String tableName, String name) { 
		int id = -1;
		try {
			Statement s = getStatement();
			ResultSet rs = s.executeQuery("select rowid from " + tableName + 
					" where name = '" + name + "'");
			if (rs.next()) { 
				id = rs.getInt(1);
			} else { 
				throw new RuntimeException("Weird exception inserted but the row is missing");
			}
			rs.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return id;
	}
	
	public PreparedStatement getInsertPS(String fluentName) throws SQLException { 
		PreparedStatement ps = _insertMap.get(fluentName);
		if (ps == null) { 
			String sql = INSERT_FLUENT.replaceAll("STUB", fluentName);
			ps = _conn.prepareStatement(sql);
			_insertMap.put(fluentName, ps);
		}
		return ps;
	}
	
	public PreparedStatement getUpdatePS(String fluentName) throws SQLException { 
		PreparedStatement ps = _updateMap.get(fluentName);
		if (ps == null) { 
			String sql = UPDATE_FLUENT.replaceAll("STUB", fluentName);
			ps = _conn.prepareStatement(sql);
			_updateMap.put(fluentName, ps);
		}
		return ps;
	}
	
	public PreparedStatement getQueryPS(String fluentName) throws SQLException { 
		PreparedStatement ps = _queryMap.get(fluentName);
		if (ps == null) { 
			String sql = QUERY_FLUENT.replaceAll("STUB", fluentName);
			ps = _conn.prepareStatement(sql);
			_queryMap.put(fluentName, ps);
		}
		return ps;
	}	
	/**
	 * Get the generated tables from the SQLite database.
	 * @param type
	 * @return
	 */
	public ResultSet getTables(String type) {
		try {
			return _conn.getMetaData().getTables(null, null, type, null);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}
	
	/**
	 * Record all of the event information into the
	 * database.
	 * @param eventName
	 * @param eventParams
	 * @param time
	 */
	public void recordEvent(String eventName, String eventParams, long time) { 
		try { 
			_eventPS.setString(1, eventName);
			_eventPS.setString(2, eventParams);
			_eventPS.setLong(3, time);
			
			_eventPS.executeUpdate();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
	
	/**
	 * Record a parameter value that will not be changing.
	 * @param name
	 * @param value
	 */
	public void recordParameter(String name, String value) { 
		try { 
			_paramsPS.setString(1, name);
			_paramsPS.setString(2, value);
			
			_paramsPS.executeUpdate();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
	
	
	public String getParameter(String name) { 
		String result = "";
		try { 
			Statement s = _conn.createStatement();
			ResultSet rs = s.executeQuery("select value from params_table where name = '" + name + "'");
			if (rs.next()) { 
				result = rs.getString(1);
			}
			
			rs.close();
			s.close();
			return result;
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return result;
	}

	public Collection<String> queryEvents(String eventName, long time) { 
		List<String> events = new ArrayList<String>();
		try { 
			Statement s = _conn.createStatement();
			ResultSet rs = s.executeQuery("select params from event_table where name = '" + eventName + "' and event_time = " + time);
			
			while (rs.next()) { 
				events.add(rs.getString(1));
			}
			
			rs.close();
			s.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return events;
	}

	public String queryEvent(String eventName, long time) { 
		String result = "";
		try { 
			Statement s = _conn.createStatement();
			ResultSet rs = s.executeQuery("select params from event_table where name = '" + eventName + "' and event_time = " + time);
			
			if (rs.next()) { 
				result = rs.getString(1);
			}
			
			rs.close();
			s.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return result;
	}
	
	public String queryFluent(String fluentName, String entitiesName, int time) {
		String result = "";
		try { 
			PreparedStatement ps = getQueryPS(fluentName);
			ps.setInt(1, findOrAddFluent(fluentName));
			ps.setInt(2, findOrAddEntityKey(entitiesName));
			ps.setInt(3, time);
			ps.setInt(4, time);
			
			ResultSet rs = ps.executeQuery();
			if (rs.next()) { 
				result = rs.getString(1);
			}
			rs.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return result;
	}
}
