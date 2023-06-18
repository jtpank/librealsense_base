CC 		= g++
FLAGS 	= -std=c++11 -Wall -pthread
LIB_RS	= realsense2
LIB_OGL	= opengl
INCLUDE = include
SOURCE	= src
OUTPUT	= bin
MAIN = main.cpp
PC = pointCloudDemo.cpp
BINARY_NAME	= mapper
BINARY_PC	= pointcloud

viewer:
	$(CC) $(FLAGS) -I$(INCLUDE) ./$(SOURCE)/$(MAIN) -o ./$(OUTPUT)/$(BINARY_NAME) -l$(LIB_RS)

pointCloud:
	$(CC) $(FLAGS) -I$(INCLUDE) ./$(SOURCE)/$(PC) -o ./$(OUTPUT)/$(BINARY_PC) -l$(LIB_RS) -lGL -lGLU -lglut -lglfw3 -ldl 

clean:
	rm -f ./$(OUTPUT)/*
