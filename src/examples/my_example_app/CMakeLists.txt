   ######################504实验室##########
      #Date:2020年9月3日
      #File name:CmakeList.txt
      #Author:sjm
      #说明：文件用于测试Pixhawk代码的编译内容
      ##################504实验室################
      PX4_add_module(
          MODULE examples__my_example_app
          MAIN my_example_app
          STACK_MAIN 2000
          SRCS
              my_example_app.c
          DEPENDS
              platforms__common
          )
