import json
import os

# Directory containing the JSON files
input_directory = "/Users/hadyi/Documents/datasets/project/json_files"
output_file = "/Users/hadyi/Documents/datasets/project/odm_geotags.txt"
lines = []

# Iterate over each JSON file in the directory
for filename in os.listdir(input_directory):
    if filename.endswith(".json"):
        # Read and parse each JSON file
        file_path = os.path.join(input_directory, filename)
        with open(file_path, "r") as json_file:
            try:
                data = json.load(json_file)
                
                # Extract necessary fields
                capture_name = os.path.splitext(filename)[0]  # Use filename without extension
                lat = data.get("lat", 0.0)
                lon = data.get("lon", 0.0)
                alt = data.get("alt", 0.0)
                yaw = data.get("yaw", 0.0)
                pitch = data.get("pitch", 0.0)
                roll = data.get("roll", 0.0)
                last_time = data.get("last_time", 0.0)
                
                # Format the line for geotags
                line = f"{capture_name},{lat},{lon},{alt},{yaw},{pitch},{roll},{last_time}"
                lines.append(line)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON file {filename}: {e}")
            except KeyError as e:
                print(f"Missing key in {filename}: {e}")

# Write all geotags to the output file
with open(output_file, "w") as output:
    output.write("\n".join(lines))

print(f"Geotags written successfully to {output_file}")
