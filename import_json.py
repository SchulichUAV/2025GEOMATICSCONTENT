# Converter script to convert multiple JSON files and their corresponding captures to a single txt file named
# 'odm_geotags.txt' for use in ODM
# This version ensures correct file encoding (UTF-8 without BOM) and Unix-style line endings.

import os
import json
import shutil

# Directory containing the JSON files
# Change with your folder path for json files
json_directory = "/Users/hadyi/Documents/datasets/code/json_files"

# Directory containing the images (ODM expects images in /datasets/images)
# Change with your folder path for image files
images_directory = "/Users/hadyi/Documents/datasets/code/images"

# Output geotags file path within the 'code' directory
output_file = "/Users/hadyi/Documents/datasets/code/odm_geotags.txt"

# ODM's expected geotags file path inside the container (to copy later)
odm_geotags_destination = "/datasets/code/odm_geotags.txt"

# Initialize lines with the EPSG declaration
lines = ["EPSG:4326"]

# Process each JSON file in the directory
for filename in os.listdir(json_directory):
    if filename.lower().endswith(".json"):
        filepath = os.path.join(json_directory, filename)
        try:
            with open(filepath, "r", encoding="utf-8") as json_file:
                data = json.load(json_file)

                # Extract fields from the JSON, with defaults
                base_name = os.path.splitext(filename)[0]
                possible_extensions = [".jpg", ".JPG", ".jpeg", ".JPEG"]
                capture_name = None

                # Find the corresponding image file with supported extensions
                for ext in possible_extensions:
                    temp_capture_name = base_name + ext
                    image_path = os.path.join(images_directory, temp_capture_name)
                    if os.path.isfile(image_path):
                        capture_name = temp_capture_name
                        break

                if not capture_name:
                    print(f"Warning: Image file for {filename} does not exist in {images_directory}. Skipping this entry.")
                    continue  # Skip if image file doesn't exist

                lat = data.get("lat", 0.0)
                lon = data.get("lon", 0.0)
                alt = data.get("rel_alt", 0.0)
                yaw = data.get("yaw", 0.0)
                pitch = data.get("pitch", 0.0)
                roll = data.get("roll", 0.0)

                # Check for NaN or invalid values
                if any([
                    not isinstance(lat, (int, float)),
                    not isinstance(lon, (int, float)),
                    not isinstance(alt, (int, float)),
                    not isinstance(yaw, (int, float)),
                    not isinstance(pitch, (int, float)),
                    not isinstance(roll, (int, float))
                ]):
                    print(f"Warning: Invalid geolocation data in file {filepath}. Skipping this entry.")
                    continue

                # Format line according to ODM requirements
                line = f"{capture_name} {lat} {lon} {alt} {yaw} {pitch} {roll}"
                lines.append(line)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON from file {filepath}: {e}")
        except Exception as e:
            print(f"Unexpected error processing file {filepath}: {e}")

# Write all lines to the output file (overwriting the old one)
try:
    with open(output_file, "w", encoding="utf-8", newline='\n') as output:
        output.write("\n".join(lines))
    print(f"Geotags written successfully to {output_file}")
except PermissionError:
    print(f"Permission denied: Unable to write to {output_file}. Check file permissions.")
except Exception as e:
    print(f"An unexpected error occurred while writing to {output_file}: {e}")

# Validate the resulting geotags file
invalid_entries = []
try:
    with open(output_file, "r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            if line_number == 1:
                # Optionally, verify that the first line is the correct EPSG declaration
                if not line.strip().startswith("EPSG:"):
                    invalid_entries.append((line_number, line.strip()))
                continue  # Skip validation for the header line

            if "NaN" in line:
                invalid_entries.append((line_number, line.strip()))
            # Check for the correct number of fields (7 fields expected)
            fields = line.strip().split()
            if len(fields) != 7:
                invalid_entries.append((line_number, line.strip()))
except FileNotFoundError:
    print(f"The output file {output_file} does not exist for validation.")
except Exception as e:
    print(f"An unexpected error occurred during validation: {e}")

# Print results of the validation check
if invalid_entries:
    print("Invalid entries found in the geotags file:")
    for line_number, entry in invalid_entries:
        print(f"Line {line_number}: {entry}")
else:
    print("No invalid entries found. Geotags file is ready for ODM.")

# **No longer copying the geotags file**, as we are placing it directly in the expected directory
