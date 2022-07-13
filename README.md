# smg-pyoctomap

This Python package provides Python bindings for Octomap.

It is a submodule of [smglib](https://github.com/sgolodetz/smglib), the open-source Python framework associated with our drone research in the [Cyber-Physical Systems](https://www.cs.ox.ac.uk/activities/cyberphysical/) group at the University of Oxford.

### Installation (as part of smglib)

Note: Please read the [top-level README](https://github.com/sgolodetz/smglib/blob/master/README.md) for smglib before following these instructions.

1. Open the terminal.

2. Build and install Octomap.

   i. Clone our fork of Octomap into `C:/octomap`:

   ```
   git clone git@github.com:sgolodetz/octomap.git C:/octomap
   ```

   ii. Change to the `C:/octomap` directory.

   iii. Check out the `heightcolouring` branch.

   iv. Configure `Octomap` using CMake, with `C:/octomap/build` as the build directory. Set relevant variables as per the box below. Then build (but don't install) `Octomap` using Visual Studio to check that it builds cleanly.

   ```
   BUILD_DYNAMICETD3D_SUBPROJECT=no
   BUILD_OCTOVIS_SUBPROJECT=no
   BUILD_TESTING=no
   CMAKE_INSTALL_PREFIX=C:/octomap/install
   ```

   v. Now go back to CMake, enable `BUILD_OCTOVIS_SUBPROJECT` and reconfigure. This will result in an error saying that libQGLViewer cannot be found or generated. Unfortunately, this must be built using Qt Creator, so you need to download Qt. The version we used was 5.14.2, the installer for which can be found [here](https://download.qt.io/archive/qt/5.14/5.14.2/). You then need to run Qt Creator and build `C:/octomap/octovis/src/extern/QGLViewer/QGLViewer.pro`.

   vi. If you go back to CMake and reconfigure, QGLViewer should now be found. However, the Qt5 directories and library paths will be detected wrongly (we want the ones from our downloaded version of Qt, not the ones in the Anaconda library). They should thus be modified accordingly (assuming that you installed Qt in `C:/Qt`):

   ```
   QT5CORE_LIBRARY_DEBUG=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/Qt5Cored.lib
   QT5CORE_LIBRARY_RELEASE=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/Qt5Core.lib
   Qt5Core_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5Core
   Qt5Gui_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5Gui
   Qt5OpenGL_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5OpenGL
   Qt5Widgets_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5Widgets
   Qt5Xml_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5Xml
   ```

   vii. Reconfigure once again, and then try regenerating the Visual Studio project. 
   
   viii. If that works, try building and installing `Octomap` in `Debug` mode. If that works, rename the `C:/octomap/install` directory to `install-debug`. Then try building and installing `Octomap` in `Release` mode. If that works, rename the `C:/octomap/install` directory `install-release`. (In both cases, use Rebuild All when doing the build to avoid issues with stale files.)

   ix. To run `C:/octomap/install-release/bin/octovis.exe`, set the `QT_PLUGIN_PATH` environment variable (at the system level) to `C:\Qt\5.14.2\5.14.2\msvc2017_64\plugins`, and copy the QGLViewer and Qt5 DLLs across to the `C:/octomap/install-release/bin` directory. Note that setting `QT_PLUGIN_PATH` in this way will temporarily prevent Qt Creator from running. To fix this, you can simply remove the `QT_PLUGIN_PATH` environment variable again (or rename it).

3. Set (at a system level, not in the terminal) the following environment variables:
   
   ```
   SMGLIB_OCTOMAP_DIR=C:/octomap
   SMGLIB_Qt5_DIR=C:/Qt/5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5
   ```

4. Re-open the terminal, and change to the `<root>/smg-pyoctomap` directory.

5. Check out the `master` branch.

6. Activate the Conda environment, e.g. `conda activate smglib`.

7. Run `pip install -e .` to install the package.

### Publications

If you build on this framework for your research, please cite the following paper:
```
@inproceedings{Golodetz2022TR,
author = {Stuart Golodetz and Madhu Vankadari* and Aluna Everitt* and Sangyun Shin* and Andrew Markham and Niki Trigoni},
title = {{Real-Time Hybrid Mapping of Populated Indoor Scenes using a Low-Cost Monocular UAV}},
booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
month = {October},
year = {2022}
}
```

### Acknowledgements

This work was supported by Amazon Web Services via the [Oxford-Singapore Human-Machine Collaboration Programme](https://www.mpls.ox.ac.uk/innovation-and-business-partnerships/human-machine-collaboration/human-machine-collaboration-programme-oxford-research-pillar), and by UKRI as part of the [ACE-OPS](https://gtr.ukri.org/projects?ref=EP%2FS030832%2F1) grant. We would also like to thank [Graham Taylor](https://www.biology.ox.ac.uk/people/professor-graham-taylor) for the use of the Wytham Flight Lab, [Philip Torr](https://eng.ox.ac.uk/people/philip-torr/) for the use of an Asus ZenFone AR, and [Tommaso Cavallari](https://uk.linkedin.com/in/tcavallari) for implementing TangoCapture.
