import json

# a Python object (dict):
x = {
    "trans_mat": {
        "box2cam": {
            "R": [0.500, 0.500, -0.500, 0.500],
            "t": [0.167, -0.717, -0.186],        
        },
        "box2bosch": {
            "R": [0.509, -0.506, -0.487, -0.498],
            "t": [0.004, -0.987, -0.117],        
        },
        "box2ouster": {
            "R": [-0.603, 0.365, 0.363, 0.609],
            "t": [-0.000, -0.050, 0.000],        
        },
        # "box2tiradar": {
        #     "R": [0.500, 0.500, -0.500, 0.500],
        #     "t": [0.087, -0.717, -0.106],        
        # },
    }
}

# convert into JSON:
with open("sensor_box_extrinsic.json", "w") as f:
    json.dump(x, f, indent=2)

with open("sensor_box_extrinsic.json", "r") as f:
    y = json.load(f)

print(y)