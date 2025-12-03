# HW4 Makefile
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

# SFML Configuration (Adjust if necessary)
SFML_LIBS := -lsfml-graphics -lsfml-window -lsfml-system

# Source files
SRCS := main.cpp graph.cpp pathfinding.cpp steering.cpp ai.cpp
OBJS := $(SRCS:.cpp=.o)
TARGET := hw4_sim

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(SFML_LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)