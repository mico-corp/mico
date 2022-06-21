\mainpage MICO 

@tableofcontents

==========================================================
What is MICO? {#what_is_mico}
==========================================================

MICO (or <b>M</b>odules-based <b>I</b>nterface for <b>C</b>omputer pr<b>O</b>gramming) is an Open-Source framework for programming without coding. The target audience of the framework is varied. It was initially designed and developed at the University of Seville by the <em>Group of Robotics Vision and Control</em>, the purpose was to create fastly applications which may have similar core concepts but with some interchangeable parts. 


The Framework is based in a lowlevel library called <b>FLOW</b>, this library is the responsible of abstracting the data streams so it is easy to interconnect different pieces of the application.

\image html flow_banner.png width=320px

The library has a QT-based wrapper that allows to interconnect create new applications dinamically without any piece of code, based on a preexisting library of modules or blocks. The QT-Based interface is a fork of the library [nodeeditor](https://github.com/paceholder/nodeeditor). We highly appreciate <em>placeholder</em> for its wonderful creation.

The rest of the framework are a set of plugins or (<em>mplugins</em>) which implement the specific blocks to be used by the visual interface <b>flow_kids</b>. The current list of modules is listed here:

1. arduino_mplugin
2. cameras_mplugin
3. core_mplugin
4. dvs_mplugin
5. fastcom_mplugin
6. math_mplugin
7. ml_mplugin
8. python_mplugin
9. robotics_mplugin
10. simulation_mplugin
11. slam_mplugin
12. visualizers_mplugin


Examples
==========================================================

\image html simple_example_1.gif width=640px

==========================================================
\image html simple_example_2.gif width=640px

==========================================================
\image html simple_example_3.gif width=640px