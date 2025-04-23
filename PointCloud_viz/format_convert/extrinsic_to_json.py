import json

# a Python object (dict):
x = {
    "date": "07-01-2025",
    "trans_mat": {
        "box2cam": {
            "R": [0.5, 0.5, -0.5, 0.5],
            "t": [0.275, -0.801, -0.155],        
        },
        "box2bosch": {
            "R": [0.509, -0.506, -0.487, -0.498],
            "t": [0.063, -0.992, -0.096],        
        },
        "box2ouster": {
            "R": [-0.603, 0.366, 0.363, 0.610],
            "t": [0.140, -0.055, -0.059],
        }
    }
}

# convert into JSON:
with open("sensor_box_extrinsic.json", "w") as f:
    json.dump(x, f, indent=2)

with open("sensor_box_extrinsic.json", "r") as f:
    y = json.load(f)

print(y)