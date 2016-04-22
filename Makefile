vis: Particles.h
	g++ -std=c++0x main.cpp Particles.cpp `pkg-config opencv --cflags` `pkg-config opencv --libs` -o sim -framework OpenGL -framework GLUT -lopencv_core -lopencv_highgui -I /usr/local/Cellar/glm/0.9.7.4/include/ -I /usr/local/Cellar/opencv/2.4.12_2/include -Wno-deprecated
