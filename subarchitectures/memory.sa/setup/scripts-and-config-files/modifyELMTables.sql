-- CAVE: THIS SEEMS TO WORK ONLY AS USER POSTGRES!!!
SELECT AddGeometryColumn('elm', 'location',  -1, 'POLYGON', 2);
CREATE INDEX location_index ON elm USING GIST (location);