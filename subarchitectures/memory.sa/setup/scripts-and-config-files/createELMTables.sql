CREATE TABLE event_type_hierarchy(
event_type		text,
sub_type		text,
PRIMARY KEY (event_type, sub_type)
);

CREATE TABLE eth_trans_closure(
event_type		text,
sub_type		text,
PRIMARY KEY (event_type, sub_type)
);
CREATE INDEX eth_tc_event_type_index ON eth_trans_closure (event_type);

CREATE TABLE elm (					-- Main table!
event_id 		serial PRIMARY KEY,
apex_event		boolean,
event_degree		integer,
event_type		text,
			-- (vorlaeufig deakt., s.o.)
			-- REFERENCES event_type_to_string (event_type_string)
			-- ON DELETE RESTRICT,     -- testen!!!
start_time		timestamp without time zone NOT NULL,	-- oder numerisch?
end_time		timestamp without time zone NOT NULL,
-- location	 	-- muss manuell hinzugef.werden
-- direction ???
CHECK (start_time <= end_time)
);

-- ACHTUNG: GEHT ANSCHEINEND NUR ALS USER POSTGRES!!!
-- siehe Datei: table-modifications-as-postgres.sql
-- evtl. eigene (event_id, location) - Tabelle wg. jdbc-Problemen?
-- SELECT AddGeometryColumn('elm', 'location',  -1, 'POLYGON', 2);
-- CREATE INDEX location_index ON elm USING GIST (location);

CREATE INDEX elm_event_type_index ON elm (event_type);
CREATE INDEX elm_start_time_index ON elm (start_time);
CREATE INDEX elm_end_time_index   ON elm (end_time);


CREATE TABLE event_specific_binary_data(
event_id		integer REFERENCES elm (event_id)
			ON DELETE CASCADE,
binary_data	        bytea NOT NULL,
PRIMARY KEY (event_id)
);
CREATE INDEX esbd_index ON event_specific_binary_data (binary_data);

CREATE TABLE event_specific_features(
event_id		integer REFERENCES elm (event_id)
			ON DELETE CASCADE,
feature_name		text NOT NULL,
feature_value		text NOT NULL,	
PRIMARY KEY (event_id, feature_name, feature_value)
-- PRIMARY KEY (event_id),
-- UNIQUE (event_id, feature_name, feature_value)
);
CREATE INDEX esf_event_id_index ON event_specific_features (event_id);
CREATE INDEX esf_feature_name_index ON event_specific_features (feature_name);
CREATE INDEX esf_feature_name_value_index ON event_specific_features (feature_name, feature_value);


CREATE TABLE sub_events(
event_id 		integer REFERENCES elm (event_id)
			ON DELETE CASCADE, 
sub_event_id 		integer REFERENCES elm (event_id)
			ON DELETE CASCADE,  
PRIMARY KEY (event_id, sub_event_id)
);
CREATE INDEX sub_events_event_id_index ON sub_events (event_id);
CREATE INDEX sub_events_sub_event_id_index ON sub_events (sub_event_id);


CREATE VIEW super_events AS
SELECT event_id AS super_event_id, 
   sub_event_id AS event_id 
FROM sub_events;

CREATE TABLE phys_entities_involved(
event_id		integer REFERENCES elm (event_id)
			ON DELETE CASCADE,
phys_entity_id		text, -- REFERENCES phys_entity_to_type (entity_id)
--			ON DELETE CASCADE, -- keine Typinformationen???
--location                -- call von postgres...
PRIMARY KEY (event_id, phys_entity_id)
-- UNIQUE (event_id, object_id)
);
CREATE INDEX pe_event_id_index ON phys_entities_involved (event_id);
CREATE INDEX pe_phys_entity_id_index ON phys_entities_involved (phys_entity_id);



-- BELOW: FOR BENCHMARKING 
CREATE TABLE bm_current(
current_config          text PRIMARY KEY
);

CREATE TABLE event_statistics(
event_id		integer, -- REFERENCES elm (event_id),
--			ON DELETE CASCADE, 
config_name		text NOT NULL,
event_type_length	integer,
subevents_no 		integer,
esbd_size		integer,
esf_no                  integer,
esf_sum                 integer
);
CREATE INDEX es_config_name_index ON event_statistics (config_name);
CREATE VIEW event_statistics_current AS 
SELECT event_statistics.* FROM event_statistics, bm_current WHERE config_name = current_config;


CREATE TABLE insertion_times(
event_id		integer PRIMARY KEY,
--                      REFERENCES elm (event_id),
--			ON DELETE CASCADE,
new_event_id		bigint,
obj_inv 		bigint,
elm 			bigint,
subevents		bigint,
apex 			bigint,
binary_data		bigint,
esf	                bigint,
commit_time		bigint,
in_store_event		bigint,
event_cnt               bigint,
run_no                  integer
);
CREATE INDEX it_run_no_index ON insertion_times (run_no);

CREATE VIEW insertion_times_current AS
SELECT insertion_times.* FROM insertion_times, event_statistics, bm_current WHERE config_name = current_config AND insertion_times.event_id = event_statistics.event_id;


CREATE TABLE query_statistics(
query_id		serial PRIMARY KEY,
config_name		text NOT NULL,
sqnumber                integer,   
min_event_id            integer,
max_event_id            integer,
match_apex              boolean,
apex                    boolean,
min_degree              integer,
max_degree              integer,
exact_type_match        boolean,
event_type		text,
time_match_mode         integer,
start_time		timestamp without time zone,	-- oder numerisch?
end_time		timestamp without time zone,
location_match_mode     integer,
esbd_size               integer,
esf_match_mode          integer,
restrict_esf_to_keys    boolean,
esf_no                  integer,
esf_sum                 integer,
subevents_match_mode    integer,
no_of_subevents         integer,
superevents_match_mode  integer,
no_of_superevents       integer,
result_size		integer,
result_limit            integer,
construction_time	bigint,
execution_time		bigint,
event_cnt               bigint,
last_event_id           integer,
src_id                  integer,
qcomment                text,
run_no                  integer
);
CREATE INDEX qs_config_name_index ON query_statistics (config_name);
CREATE INDEX qs_run_no_index ON query_statistics (run_no);

CREATE VIEW query_statistics_current AS 
SELECT query_statistics.* FROM query_statistics, bm_current WHERE config_name = current_config;



