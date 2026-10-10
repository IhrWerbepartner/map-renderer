import json
import sys

from tabulate import tabulate

if len(sys.argv) != 2:
    print("usage: %s filename" % sys.argv[0])
    exit()

bundle_file: str = sys.argv[1]

with open(bundle_file) as file:
    raw_json: str = file.read();
    json = json.loads(raw_json)

    layers = json["layers"]

    result = []

    for layer in layers:
        layer_name: str = layer["name"]
        layer_extent: int = layer["extent"]

        features = layer["features"]
        point_count = len([feature for feature in features if feature["type"] == "POINT"])
        linestring_count = len([feature for feature in features if feature["type"] == "LINESTRING"])
        polygon_count = len([feature for feature in features if feature["type"] == "POLYGON"])

        list.append(result, [layer_name, layer_extent, point_count, linestring_count, polygon_count])

    print(tabulate(result, headers=["name", "extent", "#points", "#linestrings", "#polygons"], tablefmt="orgtbl"))





