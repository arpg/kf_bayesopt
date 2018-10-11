CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本
PROJECT( PCL_TestP ) #设定工程名
SET( CMAKE_CXX_COMPILER "g++") #设定编译器
SET( CMAKE_BUILD_TYPE Release  )
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3") #这样达到使用c++11编译效果	
FIND_PACKAGE(PCL 1.5 REQUIRED)
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) #表示将产生的可执行文件全部放到bin里
	
#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
	
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)
	
#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)#这意味着在这个Cmakelists文件夹的下一个文件夹里还有cmakelists要继续构建
#ADD_EXECUTABLE(mian mian.cpp)
