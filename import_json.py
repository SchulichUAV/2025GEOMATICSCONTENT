import os
import json
import shutil
from geopy.distance import geodesic

# Directory paths
json_directory = "/Users/hadyi/Documents/datasets/code/json_files"
source_images_directory = "/Users/hadyi/Documents/datasets/images"  # Unfiltered images (from source)
filtered_images_directory = "/Users/hadyi/Documents/datasets/code/images"  # Filtered images go here
output_file = "/Users/hadyi/Documents/datasets/code/odm_geotags.txt"

# Distance threshold for filtering (in meters)
DISTANCE_THRESHOLD = 1 

# Ensure filtered images directory is clean
if os.path.exists(filtered_images_directory):
    shutil.rmtree(filtered_images_directory)
os.makedirs(filtered_images_directory, exist_ok=True)

# Initialize geotags file with EPSG declaration
lines = ["EPSG:4326"]
filtered_images = []
used_coords = []

# Process each JSON file
for filename in os.listdir(json_directory):
    if filename.lower().endswith(".json"):
        filepath = os.path.join(json_directory, filename)
        try:
            with open(filepath, "r", encoding="utf-8") as json_file:
                data = json.load(json_file)

                # Extract metadata
                base_name = os.path.splitext(filename)[0]
                possible_extensions = [".jpg", ".JPG", ".jpeg", ".JPEG"]
                capture_name = None

                # Find corresponding image in the source images folder
                for ext in possible_extensions:
                    temp_capture_name = base_name + ext
                    image_path = os.path.join(source_images_directory, temp_capture_name)
                    if os.path.isfile(image_path):
                        capture_name = temp_capture_name
                        break

                if not capture_name:
                    print(f"⚠️ Warning: Image file for {filename} not found in {source_images_directory}. Skipping.")
                    continue  

                # Extract GPS data
                lat = data.get("lat", 0.0)
                lon = data.get("lon", 0.0)
                alt = data.get("rel_alt", 0.0)
                yaw = data.get("yaw", 0.0)
                pitch = data.get("pitch", 0.0)
                roll = data.get("roll", 0.0)

                # Validate GPS
                if any(not isinstance(v, (int, float)) for v in [lat, lon, alt, yaw, pitch, roll]):
                    print(f"⚠️ Warning: Invalid geolocation data in {filename}. Skipping.")
                    continue

                # Check if image is too close to another
                keep = True
                for coord in used_coords:
                    if geodesic((lat, lon), coord).meters < DISTANCE_THRESHOLD:
                        keep = False
                        break

                if keep:
                    used_coords.append((lat, lon))
                    filtered_images.append(capture_name)
                    line = f"{capture_name} {lat} {lon} {alt} {yaw} {pitch} {roll}"
                    lines.append(line)

                    src_image_path = os.path.join(source_images_directory, capture_name)
                    dst_image_path = os.path.join(filtered_images_directory, capture_name)
                    shutil.copy2(src_image_path, dst_image_path)
                    print(f"Copied {capture_name} to {filtered_images_directory}")

        except json.JSONDecodeError as e:
            print(f"Error decoding JSON from {filepath}: {e}")
        except Exception as e:
            print(f"Unexpected error processing {filepath}: {e}")

# Ensure at least one image was copied
if not os.listdir(filtered_images_directory):
    print("No filtered images were copied. Check JSON and image paths!")
else:
    print(f"Filtered images successfully moved to {filtered_images_directory}")

# Write the filtered geotags file
try:
    with open(output_file, "w", encoding="utf-8", newline='\n') as output:
        output.write("\n".join(lines))
    print(f"Filtered geotags written to {output_file}")
except Exception as e:
    print(f"Error writing {output_file}: {e}")

# Validate odm_geotags.txt
invalid_entries = []
try:
    with open(output_file, "r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            if line_number == 1:
                continue #skip coordinate system line

            if "NaN" in line or len(line.strip().split()) != 7:
                invalid_entries.append((line_number, line.strip()))
except FileNotFoundError:
    print(f"File {output_file} does not exist for validation.")
except Exception as e:
    print(f"Unexpected error during validation: {e}")

if invalid_entries:
    print("Invalid entries found in geotags file:")
    for line_number, entry in invalid_entries:
        print(f"Line {line_number}: {entry}")
else:
    print("Geotags file is valid for ODM.")

print("Processing complete. Filtered images exist in '/datasets/code/images/'")
