#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
namespace py = pybind11;

#include <Python.h>

#include <string>

#include <octomap/octomap.h>
#include <octovis/OcTreeDrawer.h>

PYBIND11_MODULE(pyoctomap, m)
{
  // CLASSES

  py::class_<octomap::OcTree>(m, "OcTree")
    .def(py::init<double>(), py::call_guard<py::gil_scoped_release>())
    .def(py::init<std::string>(), py::call_guard<py::gil_scoped_release>())
    .def(
      "cast_ray",
      [](const octomap::OcTree& t, const octomap::point3d& origin, const octomap::point3d& direction, octomap::point3d& end,
         bool ignoreUnknownCells, double maxRange)
      {
        return t.castRay(origin, direction, end, ignoreUnknownCells, maxRange);
      },
      py::arg("origin"), py::arg("direction"), py::arg("end"), py::arg("ignoreUnknownCells") = false, py::arg("maxRange") = -1.0,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "insert_point_cloud",
      [](octomap::OcTree& t, const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
         double maxrange, bool lazy_eval, bool discretize)
      {
        t.insertPointCloud(scan, sensor_origin, maxrange, lazy_eval, discretize);
      },
      py::arg("scan"), py::arg("sensor_origin"), py::arg("maxrange") = -1.0, py::arg("lazy_eval") = false, py::arg("discretize") = false,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "insert_ray",
      [](octomap::OcTree& t, const octomap::point3d& origin, const octomap::point3d& end, double maxrange, bool lazy_eval)
      {
        return t.insertRay(origin, end, maxrange, lazy_eval);
      },
      py::arg("origin"), py::arg("end"), py::arg("maxrange") = -1.0, py::arg("lazy_eval") = false,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "is_node_occupied",
      [](const octomap::OcTree& t, const octomap::OcTreeNode& occupancyNode)
      {
        return t.isNodeOccupied(occupancyNode);
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "search",
      [](octomap::OcTree& t, const octomap::point3d& value, unsigned int depth)
      {
        return t.search(value, depth);
      },
      py::arg("value"), py::arg("depth") = 0,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "update_node",
      [](octomap::OcTree& t, const octomap::point3d& value, bool occupied, bool lazy_eval)
      {
        return t.updateNode(value, occupied, lazy_eval);
      },
      py::arg("value"), py::arg("occupied"), py::arg("lazy_eval") = false,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "write_binary",
      [](octomap::OcTree& t, const std::string& filename) { return t.writeBinary(filename); },
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomap::OcTreeDrawer>(m, "OcTreeDrawer")
    .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<octomap::OcTreeNode>(m, "OcTreeNode")
    .def("get_occupancy", &octomap::OcTreeNode::getOccupancy, py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<octomap::Pointcloud>(m, "Pointcloud")
    .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
    .def(
      "push_back", [](octomap::Pointcloud& pcd, const octomap::point3d& p) { pcd.push_back(p); },
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomath::Vector3>(m, "Vector3")
    .def_property("x", [](const octomath::Vector3& v) { return v.x(); }, [](octomath::Vector3& v, float x) { v.x() = x; })
    .def_property("y", [](const octomath::Vector3& v) { return v.y(); }, [](octomath::Vector3& v, float y) { v.y() = y; })
    .def_property("z", [](const octomath::Vector3& v) { return v.z(); }, [](octomath::Vector3& v, float z) { v.z() = z; })
    .def(
      py::init<float, float, float>(),
      py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(py::self + py::self)
    .def(py::self - py::self)
    .def(
      "__repr__",
      [](const octomath::Vector3& v)
      {
        return "( " + std::to_string(v.x()) + " " + std::to_string(v.y()) + " " + std::to_string(v.z()) + " )";
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def("copy", [](const octomath::Vector3& v) { return octomath::Vector3(v); }, py::call_guard<py::gil_scoped_release>())
    .def("norm", &octomath::Vector3::norm, py::call_guard<py::gil_scoped_release>())
    .def("rotate_ip", &octomath::Vector3::rotate_IP, py::call_guard<py::gil_scoped_release>())
  ;
}
