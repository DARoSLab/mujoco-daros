#####################################################################
# qpSWIFT MAKEFILE
#####################################################################
# Name  : Abhishek Pandala, Yanran Ding, and Hae-won Park
# Email : agp19@vt.edu, yding35@illinois.edu, haewonpark@kaist.ac.kr
# 
#
#		
# Make Targets
#	all      	: compiles and creates executable for all the problems 
#
#	
#	cplus    	: compiles the cpp interface using Eigen Input
#	RUNQP    	: compiles and creates executable for C interface
#
#	qpSWIFT.a	: compiles the qpSWIFT library
#
#	clean    	: removes all the object files
#	purge    	: removes all the object files and executables
#
#
#	Note
#	->	Compiled using gcc and g++
#	
#	->	*********** Happy Solving ***********
#
####################################################################

CC = gcc

CPP = g++

CXXFLAGS = -Wall -fPIC -fmax-errors=3 -shared

EXE = RUNQP cplus

INCLUDES = -Iinclude

EIGEN_DIR = -I/usr/local/include/eigen3 #Add the path to the eigen directory #

AMD_OBJS = amd_1.o amd_2.o amd_aat.o amd_control.o amd_defaults.o \
          amd_dump.o amd_global.o amd_info.o amd_order.o amd_post_tree.o amd_postorder.o amd_preprocess.o amd_valid.o
          

QP_OBJS = Prime.o Auxilary.o timer.o

LDL_OBJS = ldl.o




.PHONY:all clean purge
all: $(EXE)

RUNQP: libqpSWIFT.a src/RUNQP.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ src/RUNQP.c -L${CURDIR} -lqpSWIFT -lm

cplus: libqpSWIFT.a src/cplus.cpp
	$(CPP) $(CXXFLAGS) $(EIGEN_DIR) $(INCLUDES) -o $@ src/cplus.cpp -L${CURDIR} -lqpSWIFT 


libqpSWIFT.a: $(AMD_OBJS) $(LDL_OBJS) $(QP_OBJS)
	ar rcs $@ $^
	- ranlib $@
	#rm -rf *.o

%.o : src/%.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $< 




clean:
	rm -rf *.o


purge:
	rm -rf *.o *.a $(EXE)


libqpSWIFT.so: $(AMD_OBJS) $(LDL_OBJS) $(QP_OBJS)
	gcc -shared -o $@ $^ -fPIC
	sudo mv libqpSWIFT.so /usr/local/lib/

####################################################################
#	End of Makefile
####################################################################

# valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes ./RUNQP
