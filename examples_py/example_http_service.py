import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

"""
You can use fastapi to encapsulate z1_sdk for http interface call.
"""

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)

from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

class RequestDataType(BaseModel):
    func: str # backToStart, Passive and so on
    args: dict

@app.post("/unitree/z1")
def z1_server(item: RequestDataType):
    try:
        if item.func == "backToStart":
            arm.backToStart()
        elif item.func == "Passive":
            arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
        elif item.func == "labelRun":
            arm.labelRun(item.args["label"])
        elif item.func == "labelSave":
            arm.labelSave(item.args["label"])
        elif item.func == "MoveJ":
            if 'q' in item.args.keys():
                q = item.args["q"]
                Tdes = arm._ctrlComp.armModel.forwardKinematics(q, 6)
                posture = unitree_arm_interface.homoToPosture(Tdes)
            elif 'posture' in item.args.keys():
                posture = item.args["posture"]
            
            gripperPos = item.args["gripperPos"] if "gripperPos" in item.args.keys() else 0.0
            if arm.MoveJ(posture, gripperPos, item.args["maxSpeed"]):
                return {"func": item.func, "status": "success"}
            else:
                return {"func": item.func, "status": "failed"}
        elif item.func == "MoveL":
            posture = item.args["posture"]
            gripperPos = item.args["gripperPos"] if "gripperPos" in item.args.keys() else 0.0
            if arm.MoveL(posture, gripperPos, item.args["maxSpeed"]):
                return {"func": item.func, "status": "success"}
            else:
                return {"func": item.func, "status": "failed"}
        elif item.func == "MoveC":
            gripperPos = item.args["gripperPos"] if "gripperPos" in item.args.keys() else 0.0
            middlePosture = item.args["middlePosture"]
            endPosture = item.args["endPosture"]
            if arm.MoveC(middlePosture, endPosture, gripperPos, item.args["maxSpeed"]):
                return {"func": item.func, "status": "success"}
            else:
                return {"func": item.func, "status": "failed"}
        elif item.func == "getQ":
            return {"q": arm.lowstate.getQ().tolist()}
    except Exception as e:
        return {"func": item.func, "status": "failed", "error": str(e)}

    return {"func": item.func} # some function without return state

arm.loopOn()
import uvicorn
uvicorn.run(app, host="0.0.0.0", port=12000)
arm.loopOff()
