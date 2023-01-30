#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

class ArmInterface : public unitreeArm
{
public:
    ArmInterface(bool hasGripper):unitreeArm(hasGripper){};
    ~ArmInterface(){};
    void loopOn() { sendRecvThread->start();}
    void loopOff() { sendRecvThread->shutdown();}
    void setFsmLowcmd()
    {
        sendRecvThread->start();
        setFsm(ArmFSMState::PASSIVE);
        setFsm(ArmFSMState::LOWCMD);
        sendRecvThread->shutdown();
    }
};

namespace py = pybind11;
PYBIND11_MODULE(unitree_arm_interface, m){
    py::enum_<ArmFSMState>(m, "ArmFSMState")
        .value("INVALID", ArmFSMState::INVALID)
        .value("PASSIVE", ArmFSMState::PASSIVE)
        .value("LOWCMD", ArmFSMState::LOWCMD)
        .value("JOINTCTRL", ArmFSMState::JOINTCTRL)
        .value("CARTESIAN", ArmFSMState::CARTESIAN)
        ;

    py::class_<LowlevelState>(m, "LowlevelState")
        .def("getQ", &LowlevelState::getQ, py::return_value_policy::reference_internal)
        .def("getQd", &LowlevelState::getQd, py::return_value_policy::reference_internal)
        .def("getQdd", &LowlevelState::getQdd, py::return_value_policy::reference_internal)
        .def("getQTau", &LowlevelState::getTau, py::return_value_policy::reference_internal)
        ;

    py::class_<ArmInterface>(m, "ArmInterface")
        .def(py::init<bool>())
        .def_readwrite("q", &ArmInterface::q)
        .def_readwrite("qd", &ArmInterface::qd)
        .def_readwrite("tau", &ArmInterface::tau)
        .def_readwrite("gripperQ", &ArmInterface::gripperQ)
        .def_readwrite("gripperQd", &ArmInterface::gripperW)
        .def_readwrite("gripperTau", &ArmInterface::gripperTau)
        .def_readwrite("lowstate", &ArmInterface::lowstate)
        .def_readwrite("_ctrlComp", &ArmInterface::_ctrlComp)
        .def("setFsmLowcmd", &ArmInterface::setFsmLowcmd)
        .def("loopOn", &ArmInterface::loopOn)
        .def("loopOff", &ArmInterface::loopOff)
        .def("setFsm", &ArmInterface::setFsm)
        .def("backToStart", &ArmInterface::backToStart)
        .def("labelRun", &ArmInterface::labelRun)
        .def("labelSave", &ArmInterface::labelSave)
        .def("teach", &ArmInterface::teach)
        .def("teachRepeat", &ArmInterface::teachRepeat)
        .def("calibration", &ArmInterface::calibration)
        .def("MoveJ", py::overload_cast<Vec6, double>(&ArmInterface::MoveJ))
        .def("MoveJ", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveJ))
        .def("MoveL", py::overload_cast<Vec6, double>(&ArmInterface::MoveL))
        .def("MoveL", py::overload_cast<Vec6, double, double>(&ArmInterface::MoveL))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double>(&ArmInterface::MoveC))
        .def("MoveC", py::overload_cast<Vec6, Vec6, double, double>(&ArmInterface::MoveC))
        .def("startTrack", &ArmInterface::startTrack)
        .def("sendRecv", &ArmInterface::sendRecv)
        .def("setWait", &ArmInterface::setWait)
        .def("jointCtrlCmd", &ArmInterface::jointCtrlCmd)
        .def("cartesianCtrlCmd", &ArmInterface::cartesianCtrlCmd)
        ;
}