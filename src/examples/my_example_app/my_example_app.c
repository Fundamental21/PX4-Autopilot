    /****************504实验室*************************
      Date:2020年9月3日
      File name:my_example_app.c
      Author:sjm
      说明：文件用于测试Pixhawk代码
      ********************************************************/
      #include <px4_posix.h>//包含了打印信息函数：PX4_INFO
      __EXPORT int my_example_app_main(int argc, char *argv[]);

      int my_example_app_main(int argc, char *argv[])
      {
          PX4_INFO("Hello Sky!");
          return OK;
      }
