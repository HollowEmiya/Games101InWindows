# Games101作业问题(在Visual Studio下)

* 作业前先修改`CMake`文件：

  * 添加`Eigen3`的路径：

    > include_directories("C:\\Program Files (x86)\\Eigen3\\include")

  * 在菜单栏>项目>`CMake`属性里，设置OpenCv的路径：

    > D:/biancheng/Game/Game101/Game101.Soft/OpenCv/opencv/build

* 有的时候相对路径会失效，可将素材改为绝对路径：

  比如，这个错误：

  <img src="D:\biancheng\Game\Game101\Game101.HomeWork\work\error_one.png" alt="error_one" style="zoom:80%;" />

  此为作业3，`勇敢牛牛`的错误需要修改

* `2021.11.27`发现未加载C++项目错误，但是未发现解决办法，似乎不影响编写。

* 严重性	代码	说明	项目	文件	行	禁止显示状态
  错误		CMake Error at D:\biancheng\Game\Game101\Game101.HomeWork\work\Assignment3\Code\CMakeLists.txt:5 (find_package):
    Found package configuration file:

      D:/biancheng/Game/Game101/Game101.Soft/OpenCv/opencv/build/OpenCVConfig.cmake

    but it set OpenCV_FOUND to FALSE so package "OpenCV" is considered to be
    NOT FOUND.	Rasterizer	D:\biancheng\Game\Game101\Game101.HomeWork\work\Assignment3\Code\CMakeLists.txt	5	