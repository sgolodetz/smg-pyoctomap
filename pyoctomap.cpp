#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
namespace py = pybind11;

#include <Python.h>

#include <string>

// Note: This is evil, but the only way I know of to get at Pointcloud::points to resize it without changing Octomap itself.
#define protected public

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octovis/OcTreeDrawer.h>

PYBIND11_MODULE(pyoctomap, m)
{
  // CLASSES

  py::class_<octomap::ColorOcTree>(m, "ColorOcTree")
    .def(py::init<double>(), py::call_guard<py::gil_scoped_release>())
    .def(
      "insert_point_cloud",
      [](octomap::ColorOcTree& self, const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
         double maxrange, bool lazy_eval, bool discretize)
      {
        self.insertPointCloud(scan, sensor_origin, maxrange, lazy_eval, discretize);
      },
      py::arg("scan"), py::arg("sensor_origin"), py::arg("maxrange") = -1.0, py::arg("lazy_eval") = false, py::arg("discretize") = false,
      py::call_guard<py::gil_scoped_release>()
    )
    .def("prune", &octomap::ColorOcTree::prune, py::call_guard<py::gil_scoped_release>())
    .def(
      "search",
      [](octomap::ColorOcTree& self, const octomap::point3d& value, unsigned int depth)
      {
        return self.search(value, depth);
      },
      py::arg("value"), py::arg("depth") = 0,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def("size", &octomap::ColorOcTree::size, py::call_guard<py::gil_scoped_release>())
    .def("update_inner_occupancy", &octomap::ColorOcTree::updateInnerOccupancy, py::call_guard<py::gil_scoped_release>())
    .def(
      "update_node",
      [](octomap::ColorOcTree& self, const octomap::point3d& value, bool occupied, bool lazy_eval)
      {
        return self.updateNode(value, occupied, lazy_eval);
      },
      py::arg("value"), py::arg("occupied"), py::arg("lazy_eval") = false,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "write",
      [](octomap::ColorOcTree& self, const std::string& filename)
      {
        return self.write(filename);
      },
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomap::ColorOcTreeNode>(m, "ColorOcTreeNode")
    .def(
      "set_color",
      [](octomap::ColorOcTreeNode& self, uint8_t r, uint8_t g, uint8_t b)
      {
        self.setColor(r, g, b);
      },
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomap::OcTree>(m, "OcTree")
    .def(py::init<double>(), py::call_guard<py::gil_scoped_release>())
    .def(py::init<std::string>(), py::call_guard<py::gil_scoped_release>())
    .def(
      "cast_ray",
      [](const octomap::OcTree& self, const octomap::point3d& origin, const octomap::point3d& direction, octomap::point3d& end,
         bool ignoreUnknownCells, double maxRange)
      {
        return self.castRay(origin, direction, end, ignoreUnknownCells, maxRange);
      },
      py::arg("origin"), py::arg("direction"), py::arg("end"), py::arg("ignoreUnknownCells") = false, py::arg("maxRange") = -1.0,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "delete_node",
      [](octomap::OcTree& self, const octomap::point3d& value, unsigned int depth)
      {
        return self.deleteNode(value, depth);
      },
      py::arg("value"), py::arg("depth") = 0,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "get_occupancy_thres",
      [](const octomap::OcTree& self)
      {
        return self.getOccupancyThres();
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "get_resolution",
      [](const octomap::OcTree& self)
      {
        return self.getResolution();
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "insert_point_cloud",
      [](octomap::OcTree& self, const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
         double maxrange, bool lazy_eval, bool discretize)
      {
        self.insertPointCloud(scan, sensor_origin, maxrange, lazy_eval, discretize);
      },
      py::arg("scan"), py::arg("sensor_origin"), py::arg("maxrange") = -1.0, py::arg("lazy_eval") = false, py::arg("discretize") = false,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "insert_ray",
      [](octomap::OcTree& self, const octomap::point3d& origin, const octomap::point3d& end, double maxrange, bool lazy_eval)
      {
        return self.insertRay(origin, end, maxrange, lazy_eval);
      },
      py::arg("origin"), py::arg("end"), py::arg("maxrange") = -1.0, py::arg("lazy_eval") = false,
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "is_node_occupied",
      [](const octomap::OcTree& self, const octomap::OcTreeNode& occupancyNode)
      {
        return self.isNodeOccupied(occupancyNode);
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "search",
      [](octomap::OcTree& self, const octomap::point3d& value, unsigned int depth)
      {
        return self.search(value, depth);
      },
      py::arg("value"), py::arg("depth") = 0,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "set_node_value",
      [](octomap::OcTree& self, const octomap::point3d& value, float log_odds_value, bool lazy_eval)
      {
        return self.setNodeValue(value, log_odds_value, lazy_eval);
      },
      py::arg("value"), py::arg("log_odds_value"), py::arg("lazy_eval") = false,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "set_occupancy_thres",
      [](octomap::OcTree& self, double prob)
      {
        self.setOccupancyThres(prob);
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "update_node",
      [](octomap::OcTree& self, const octomap::point3d& value, bool occupied, bool lazy_eval)
      {
        return self.updateNode(value, occupied, lazy_eval);
      },
      py::arg("value"), py::arg("occupied"), py::arg("lazy_eval") = false,
      py::return_value_policy::reference, py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "write_binary",
      [](octomap::OcTree& self, const std::string& filename)
      {
        return self.writeBinary(filename);
      },
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomap::OcTreeDrawer>(m, "OcTreeDrawer")
    .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
    .def("draw", &octomap::OcTreeDrawer::draw, py::call_guard<py::gil_scoped_release>())
    .def(
      "enable_freespace",
      &octomap::OcTreeDrawer::enableFreespace,
      py::arg("enabled") = true,
      py::call_guard<py::gil_scoped_release>()
    )
    .def("set_color_mode", &octomap::OcTreeDrawer::setColorMode, py::call_guard<py::gil_scoped_release>())
    .def(
      "set_octree",
      [](octomap::OcTreeDrawer& self, const octomap::OcTree& octree, const octomap::pose6d& origin, int map_id_)
      {
        self.setOcTree(octree, origin, map_id_);
      },
      py::arg("octree"), py::arg("origin"), py::arg("map_id_") = 0,
      py::call_guard<py::gil_scoped_release>()
    )
  ;

  py::class_<octomap::OcTreeNode>(m, "OcTreeNode")
    .def("get_occupancy", &octomap::OcTreeNode::getOccupancy, py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<octomap::Pointcloud>(m, "Pointcloud", pybind11::buffer_protocol())
    .def_buffer([](octomap::Pointcloud& pcd) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        &pcd[0](0),
        sizeof(float),
        pybind11::format_descriptor<float>::format(),
        1,
        { pcd.size() * 3 },
        { sizeof(float) }
      );
    })
    .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
    .def(
      "__getitem__",
      [](const octomap::Pointcloud& self, size_t i)
      {
        return self[i];
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "__setitem__",
      [](octomap::Pointcloud& self, size_t i, const octomath::Vector3& v)
      {
        self[i] = v;
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def(
      "push_back",
      [](octomap::Pointcloud& self, const octomap::point3d& p)
      {
        self.push_back(p);
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def("reserve", &octomap::Pointcloud::reserve, py::call_guard<py::gil_scoped_release>())
    .def(
      "resize",
      [](octomap::Pointcloud& self, size_t newSize)
      {
        self.points.resize(newSize);
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def("size", &octomap::Pointcloud::size, py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<octomath::Pose6D>(m, "Pose6D")
    .def(py::init<float, float, float, double, double, double>(), py::call_guard<py::gil_scoped_release>())
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
    .def(py::self * float())
    .def(py::self - py::self)
    .def(
      "__repr__",
      [](const octomath::Vector3& self)
      {
        return "( " + std::to_string(self.x()) + " " + std::to_string(self.y()) + " " + std::to_string(self.z()) + " )";
      },
      py::call_guard<py::gil_scoped_release>()
    )
    .def("copy", [](const octomath::Vector3& self) { return octomath::Vector3(self); }, py::call_guard<py::gil_scoped_release>())
    .def("norm", &octomath::Vector3::norm, py::call_guard<py::gil_scoped_release>())
    .def("rotate_ip", &octomath::Vector3::rotate_IP, py::call_guard<py::gil_scoped_release>())
  ;

  // ENUMERATIONS

  py::enum_<octomap::SceneObject::ColorMode>(m, "EColorMode")
    .value("CM_FLAT", octomap::SceneObject::CM_FLAT)
    .value("CM_PRINTOUT", octomap::SceneObject::CM_PRINTOUT)
    .value("CM_COLOR_HEIGHT", octomap::SceneObject::CM_COLOR_HEIGHT)
    .value("CM_GRAY_HEIGHT", octomap::SceneObject::CM_GRAY_HEIGHT)
    .value("CM_SEMANTIC", octomap::SceneObject::CM_SEMANTIC)
    .export_values()
  ;
}
