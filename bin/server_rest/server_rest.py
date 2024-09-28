from flask import Flask, jsonify
import json
import argparse

app = Flask(__name__)

@app.route('/')
def serve_json():
  with open(file_name+'.json', 'r') as f:
    data = json.load(f)

  return jsonify(data)

def parse_opt():
    parser = argparse.ArgumentParser(description="LDC2024 JSON Server")
    required = parser.add_argument_group("required arguments")
    required.add_argument(
        "--ip",
        type=str,
        help="This is the jury pc IP, e.g. 192.168.1.1",
    )
    required.add_argument(
        "--port",
        type=str,
        help="This is the selected port, e.g. 8080",
    )
    required.add_argument(
        "--file_name",
        type=str,
        help="This is the name of the mission file without .json, e.g. milano_manche1",
    )
    opt = parser.parse_args()
    return opt


global file_name
if __name__ == '__main__':
  opt = parse_opt()
  file_name = opt.file_name
  app.run(debug=True, host=opt.ip, port=opt.port)
