package edu.arizona.cs.learn.timeseries.prep.ww2d;


public interface StateDatabaseI {
	public final String CREATE_FLUENT_LOOKUP_TABLE =
		"create table if not exists lookup_fluent_table (" +
		"name TEXT" +
		")";
	
	public final String CREATE_ENTITY_LIST_TABLE = 
		"create table if not exists entity_map_table (" +
		"name TEXT" + 
		")";
	
	public final String CREATE_EVENT_TABLE =
		"create table if not exists event_table (" + 
		"name TEXT, " + 
		"params TEXT, " +
		"event_time INTEGER " +
		")";
		
	public final String CREATE_PARAMS_TABLE =
		"create table if not exists params_table (" + 
		"name TEXT, " + 
		"value TEXT " +
		")";
	
	public final String CREATE_FLUENT_TABLE = 
		"create table if not exists fluent_STUB (" +
		"fluent_id INTEGER, " +  
		"entities_id INTEGER, " + 
		"value TEXT, " + 
		"start_time INTEGER, " +
		"end_time INTEGER " +
		")";

	public final String CREATE_FLUENT_INDEX_1 = 
		"create index if not exists lookup_index_STUB_1 " + 
		"on fluent_STUB (session_id, fluent_id, entities_id)";
	
	public final String INSERT_FLUENT = 
		"insert into fluent_STUB (rowid, fluent_id, entities_id, value, start_time, end_time) " +
		"values (?, ?, ?, ?, ?, 0)";
	
	public final String UPDATE_FLUENT = 
		"update fluent_STUB set end_time = ? " + 
		" where rowid = ? ";
	
	public final String QUERY_FLUENT =
		"select value from fluent_STUB " +
		"where fluent_id = ? " +
		"  and entities_id = ? " +
		"  and start_time <= ? " +
		"  and end_time > ?";
	
	public final String INSERT_EVENT = 
		"insert into event_table (name, params, event_time) values (?, ?, ?)";
	
	public final String INSERT_PARAMS = 
		"insert into params_table (name, value) values (?, ?)";
	
}	
