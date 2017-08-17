# Declaration of variables
CC = g++
CC_FLAGS = -w -std=c++11 -g -O0 -I../include -I../include/librcsscontroller
 
# File names
EXEC = findballexp
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

# Main target
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC) -L../lib -lrcsscontroller
 
# To obtain object files
%.o: %.cpp
	$(CC) -c $(CC_FLAGS) $< -o $@
 
# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)