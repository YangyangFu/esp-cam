import json
import sys
import os

arduino_json_path = os.path.join(sys.argv[1], ".vscode", "arduino.json")
relative_file = sys.argv[2]

with open(arduino_json_path, "r") as f:
    data = json.load(f)

data["sketch"] = relative_file

with open(arduino_json_path, "w") as f:
    json.dump(data, f, indent=4)

print(f"Sketch set to: {relative_file}")
