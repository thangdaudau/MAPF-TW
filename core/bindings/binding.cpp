#include <pybind11/pybind11.h>
#include <pybind11/stl.h> 
#include <Python.h> /* Cần thư viện này để gọi PyErr_CheckSignals */

#include "CBSBased/CBS.h"
#include "CBSBased/ICBS.h"
#include "CBSBased/GCBS.h"
#include "base.h"
#include <utility>

namespace py = pybind11;
using namespace Core;

// Định nghĩa biến toàn cục (đã khai báo extern trong base.h)
// Core::PythonSignals Core::g_pythonSignals = nullptr;

using SolutionResult = std::pair<double, std::vector<Core::Path>>;

// --- HÀM TRỢ GIÚP AN TOÀN (Thread-Safe Wrapper) ---
// Hàm này dùng khi C++ đang chạy đa luồng (đã nhả GIL).
// Nó tự động lấy lại GIL tạm thời để kiểm tra, sau đó trả lại ngay.
// Signature: int(*)() -> Khớp hoàn toàn với PyErr_CheckSignals để template C++ hoạt động.
int SafeSignalCheck() {
    py::gil_scoped_acquire acquire; // Lấy GIL
    return PyErr_CheckSignals();    // Gọi API Python an toàn
} // Tự động nhả GIL khi hết scope

PYBIND11_MODULE(mapf_solver, m) {
    m.doc() = "MAPF Solver Wrapper";

    // --- Binding Enums ---
    py::enum_<GCBSMode>(m, "GCBSMode")
        .value("GCBS_H", GCBSMode::GCBS_H)
        .value("GCBS_L", GCBSMode::GCBS_L)
        .value("GCBS_LH", GCBSMode::GCBS_LH)
        .export_values();
        
    py::enum_<ConflictHeuristic>(m, "ConflictHeuristic")
        .value("NUM_CONFLICTS", ConflictHeuristic::NUM_CONFLICTS)
        .value("NUM_CONFLICTING_AGENTS", ConflictHeuristic::NUM_CONFLICTING_AGENTS)
        .value("NUM_CONFLICTING_PAIRS", ConflictHeuristic::NUM_CONFLICTING_PAIRS)
        .value("VERTEX_COVER", ConflictHeuristic::VERTEX_COVER)
        .value("ALTERNATING", ConflictHeuristic::ALTERNATING)
        .export_values();
        
    // --- Binding CBS ---
    py::class_<CBS>(m, "CBS")
        .def(py::init<int, int, std::vector<std::pair<int, int>>, std::vector<std::tuple<int, int, int, int>>>())
        .def("set_time_limit", &CBS::setTimeLimit)
        .def("solve", [](CBS& self, bool release_gil) -> SolutionResult {
            
            if (release_gil) {
                // OPTION 1: Nhả GIL để chạy nhanh/song song
                py::gil_scoped_release release; 
                // Phải dùng hàm bọc SafeSignalCheck để tránh crash
                self.solve(&SafeSignalCheck);
            } else {
                // OPTION 2: Giữ GIL (chặn luồng Python)
                // Dùng trực tiếp hàm của Python vì đang cầm GIL
                self.solve(&PyErr_CheckSignals);
            }

            // Kiểm tra xem có lệnh dừng (Ctrl+C) đang chờ không để ném Exception ra Python
            if (PyErr_Occurred()) {
                throw py::error_already_set();
            }

            if (self.result.found) {
                return {self.result.cost, self.result.solution};
            } else {
                return {};
            }
        }, py::arg("release_gil") = false); // Mặc định là False (Giữ GIL)
        
    // --- Binding ICBS ---
    py::class_<ICBS>(m, "ICBS")
        .def(py::init<int, int, 
             std::vector<std::pair<int, int>>, 
             std::vector<std::tuple<int, int, int, int>>,
             int, bool, uint32_t>(),
             py::arg("_row"), py::arg("_col"), 
             py::arg("_edges"), py::arg("_agents"),
             py::arg("_mergeThreshold") = 25,
             py::arg("_mergeRestartActive") = true,
             py::arg("_maxMetaAgentSize") = (uint32_t)-1)
        .def("set_time_limit", &ICBS::setTimeLimit)
        .def("solve", [](ICBS& self, bool release_gil) -> SolutionResult {
            
            // Logic tương tự CBS
            if (release_gil) {
                py::gil_scoped_release release;
                self.solve(&SafeSignalCheck);
            } else {
                self.solve(PyErr_CheckSignals);
            }

            if (PyErr_Occurred()) throw py::error_already_set();

            if (self.result.found) {
                return {self.result.cost, self.result.solution};
            } else {
                return {};
            }
        }, py::arg("release_gil") = false);

    // --- Binding GCBS ---
    py::class_<GCBS>(m, "GCBS")
        .def(py::init<int, int, 
             std::vector<std::pair<int, int>>, 
             std::vector<std::tuple<int, int, int, int>>,
             GCBSMode, ConflictHeuristic>(),
             py::arg("_row"), py::arg("_col"), 
             py::arg("_edges"), py::arg("_agents"),
             py::arg("_mode") = GCBSMode::GCBS_LH,
             py::arg("_heuristicType") = ConflictHeuristic::NUM_CONFLICTING_PAIRS)
        .def("set_time_limit", &GCBS::setTimeLimit)
        .def("solve", [](GCBS& self, bool release_gil) -> SolutionResult {
            
            if (release_gil) {
                py::gil_scoped_release release;
                self.solve(&SafeSignalCheck);
            } else {
                self.solve(PyErr_CheckSignals);
            }

            if (PyErr_Occurred()) throw py::error_already_set();

            if (self.result.found) {
                return {self.result.cost, self.result.solution};
            } else {
                return {};
            }
        }, py::arg("release_gil") = false); 
}