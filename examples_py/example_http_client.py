import requests
import numpy as np
np.set_printoptions(precision=3)

url = "http://127.0.0.1:12000/unitree/z1"
database = {
    "func": "",
    "args": {},
}

def labelRun(label):
    assert len(label) < 10

    # copy data
    data = database.copy()
    data["func"] = "labelRun"
    data["args"] = {
        "label": label,
    }
    return requests.post(url, json=data)

def labelSave(label):
    assert len(label) < 10

    # copy data
    data = database.copy()
    data["func"] = "labelSave"
    data["args"] = {
        "label": label,
    }
    return requests.post(url, json=data)

def backToStart():
    data = database.copy()
    data["func"] = "backToStart"
    return requests.post(url, json=data)

def Passive():
    data = database.copy()
    data["func"] = "Passive"
    return requests.post(url, json=data)

def getQ():
    data = database.copy()
    data["func"] = "getQ"
    return requests.post(url, json=data)
    
def MoveJ(q: list, gripperPos = 0, speed = 0.5):
    assert len(q) == 6

    data = database.copy()
    data["func"] = "MoveJ"
    data["args"] = {
        "q": q,
        "gripperPos": gripperPos,
        "maxSpeed": speed,
    }
    return requests.post(url, json=data)

# test
labelRun("forward")
backToStart()
Passive()