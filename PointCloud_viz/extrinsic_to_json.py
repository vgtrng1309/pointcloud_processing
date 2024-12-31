import json

# a Python object (dict):
x = {
    "trans_mat": {
        # "box2cam": {
        #     "R": [0.500, 0.500, -0.500, 0.500],
        #     "t": [0.167, -0.717, -0.186],        
        # },
        "box2cam": {
            "R": [0.5, 0.5, -0.5, 0.5],
            "t": [0.250, -0.761, -0.155],        
        },
        "box2bosch": {
            "R": [0.509, -0.506, -0.487, -0.498],
            "t": [0.043, -0.952, -0.096],        
        },
        "box2ouster": {
            "R": [-0.603, 0.366, 0.363, 0.610],
            "t": [0.120, -0.015, -0.059],  #0.080, -0.050, -0.080
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