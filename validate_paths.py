import json
import os
from math import sqrt

class Translation2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __str__(self):
        return f"({self.x:.3f}, {self.y:.3f})"
    
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def equals(self, other, tolerance=0.001):
        return self.distance(other) < tolerance

def validate_paths():
    path_dir = "src/main/deploy/pathplanner/paths"
    
    if not os.path.exists(path_dir) or not os.path.isdir(path_dir):
        print(f"ERROR: Path directory not found: {os.path.abspath(path_dir)}")
        return
    
    path_files = [f for f in os.listdir(path_dir) 
                  if ((f.startswith("Station to") or f.endswith("to Station.path")) 
                  and f.endswith(".path")
                  and not f.startswith("Invert"))]
    
    if not path_files:
        print(f"WARNING: No path files matching the pattern found in {os.path.abspath(path_dir)}")
        return
    
    print(f"Found {len(path_files)} path files to validate")
    
    location_coordinates = {}
    
    for file_name in path_files:
        file_path = os.path.join(path_dir, file_name)
        try:
            with open(file_path, 'r') as file:
                path_data = json.load(file)
            
            # Check the version
            version = path_data.get("version", "unknown")
            print(f"Processing {file_name} (version: {version})...")
            
            # Get waypoints
            if "waypoints" not in path_data:
                print(f"WARNING: No waypoints in {file_name}")
                continue
                
            waypoints = path_data["waypoints"]
            if len(waypoints) < 2:
                print(f"WARNING: Not enough waypoints in {file_name}")
                continue
            
            # Extract first and last positions based on the file format
            first_waypoint = waypoints[0]
            last_waypoint = waypoints[-1]
            
            # 2025.0 format uses "anchor" instead of "anchorPoint"
            if "anchor" in first_waypoint:
                first_pos = first_waypoint["anchor"]
                last_pos = last_waypoint["anchor"]
            elif "anchorPoint" in first_waypoint:
                first_pos = first_waypoint["anchorPoint"]
                last_pos = last_waypoint["anchorPoint"]
            else:
                print(f"WARNING: Can't find position data in {file_name}")
                continue
            
            first_translation = Translation2d(first_pos.get("x", 0), first_pos.get("y", 0))
            last_translation = Translation2d(last_pos.get("x", 0), last_pos.get("y", 0))
            
            print(f"  First point: {first_translation}")
            print(f"  Last point: {last_translation}")
            
            if file_name.startswith("Station to"):
                destination = file_name[len("Station to "):-len(".path")]
                check_and_store_coordinates(location_coordinates, "Station", first_translation)
                check_and_store_coordinates(location_coordinates, destination, last_translation)
            elif file_name.endswith("to Station.path"):
                source = file_name[:-len(" to Station.path")]
                check_and_store_coordinates(location_coordinates, source, first_translation)
                check_and_store_coordinates(location_coordinates, "Station", last_translation)
            
        except Exception as e:
            print(f"Error processing {file_name}: {str(e)}")
            import traceback
            traceback.print_exc()
    
    # Print all identified locations and coordinates
    print("\nLocation coordinates:")
    for location, coords in location_coordinates.items():
        print(f"{location}: {coords}")
    
    # Check if "Station" coordinates are consistent
    if "Station" in location_coordinates:
        print("\nStation coordinates check:")
        station_coords = location_coordinates["Station"]
        print(f"Station: {station_coords}")
        
        # Check all paths that connect to station
        inconsistencies = 0
        for file_name in path_files:  # Already filtered to exclude "Invert" paths
            if file_name.startswith("Station to") or file_name.endswith("to Station.path"):
                with open(os.path.join(path_dir, file_name), 'r') as file:
                    path_data = json.load(file)
                
                waypoints = path_data["waypoints"]
                
                # Get the station position from this file
                if file_name.startswith("Station to"):
                    # Station is the first point
                    waypoint = waypoints[0]
                elif file_name.endswith("to Station.path"):
                    # Station is the last point
                    waypoint = waypoints[-1]
                
                if "anchor" in waypoint:
                    pos = waypoint["anchor"]
                else:
                    pos = waypoint["anchorPoint"]
                
                file_station = Translation2d(pos.get("x", 0), pos.get("y", 0))
                if not file_station.equals(station_coords):
                    print(f"  Mismatch in {file_name}: {file_station} (should be {station_coords})")
                    inconsistencies += 1
        
        if inconsistencies == 0:
            print("All station coordinates are consistent!")
        else:
            print(f"Found {inconsistencies} inconsistencies in station coordinates.")

def check_and_store_coordinates(coord_map, location, coords):
    if location in coord_map:
        stored_coords = coord_map[location]
        distance = coords.distance(stored_coords)
        
        if not coords.equals(stored_coords):
            print(f"WARNING: Inconsistent coordinates for {location}")
            print(f"  Previous: {stored_coords}")
            print(f"  Current: {coords}")
            print(f"  Difference: {distance:.3f} meters")
    else:
        coord_map[location] = coords

if __name__ == "__main__":
    validate_paths()