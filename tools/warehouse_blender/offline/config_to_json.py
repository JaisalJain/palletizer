"""cell_config.yaml -> cell_config.json (Blender's Python has no PyYAML).
    python config_to_json.py <cell_config.yaml> <out.json>"""
import sys, json, yaml
json.dump(yaml.safe_load(open(sys.argv[1])), open(sys.argv[2], "w"), indent=1)
