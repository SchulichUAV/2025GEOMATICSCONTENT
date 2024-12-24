import json
import os

# Specify the directory containing the JSON files
input_directory = "/Users/hadyi/Documents/datasets/project/json_files"
output_file = "odm_geotags.txt"
lines = []

# Iterate over each JSON file in the directory
for filename in os.listdir(input_directory):
    if filename.endswith(".json"):
        capture_name = os.path.splitext(filename)[0]
        
        # Read and parse the JSON file
        file_path = os.path.join(input_directory, filename)
        with open(file_path, "r") as json_file:
            data = json.load(json_file)
            
            lat = data.get("lat", 0.0)
            lon = data.get("lon", 0.0)
            alt = data.get("alt", 0.0)
            roll = data.get("roll", 0.0)
            pitch = data.get("pitch", 0.0)
            yaw = data.get("yaw", 0.0)
            last_time = data.get("last_time", 0.0)
            
            line = f"{capture_name},{lat},{lon},{alt},{yaw},{pitch},{roll},{last_time}"
            lines.append(line)

with open(output_file, "w") as output:
    output.write("\n".join(lines))

print(f"Geotags written successfully to {output_file}")
