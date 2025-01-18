# converter script to convert a number of json files and their corresponding captures to a single txt named
# 'odm_geotags that can be used in odm
# current status of odm: Odm can read the geotags file and each captures coordinates in the txt, but fails to locate its geoposition.
# resulting in inaccurate mapping

# start of code
import json
import os

# Specify the directory containing the JSON files
filename = "./output.json"
output_file = "odm_geotags.txt"
lines = []

with open(filename, "r") as json_file:
    data = json.load(json_file)

    for object in data:

        capture_name = object.get("capture_name")
        lat = object.get("lat")
        lon = object.get("lon")
        alt = object.get("alt", 0.0)
        yaw = object.get("yaw", 0.0)
        pitch = object.get("pitch", 0.0)
        roll = object.get("roll", 0.0)

        line = f"{capture_name},{lat},{lon},{alt},{yaw},{pitch},{roll},{last_time}"
        lines.append(line)

with open(output_file, "w") as output:
    output.write("\n".join(lines))

print(f"Geotags written successfully to {output_file}")
