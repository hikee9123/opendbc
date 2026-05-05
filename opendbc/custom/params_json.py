import json
from openpilot.common.params import Params

def read_json_file(file_path: str):
    json_object = {}

    params = Params()
    json_str = params.get(file_path)

    if not json_str:
        return json_object

    try:
        json_object = json.loads(json_str)
    except json.JSONDecodeError as e:
        print(f"Failed to parse the JSON document: {file_path}")
        print(e)
    
    return json_object

