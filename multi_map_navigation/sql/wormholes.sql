-- Create wormholes table
CREATE TABLE IF NOT EXISTS wormholes (
    map_from TEXT,
    map_to TEXT,
    x REAL,
    y REAL,
    z REAL,
    yaw REAL
);

-- Wormholes between room1 and room2
INSERT INTO wormholes VALUES ('room1', 'room2', 0.156, 0.399, 0.0, -3.03);
INSERT INTO wormholes VALUES ('room2', 'room1', 0.156, 0.399, 0.0, 3.14);


