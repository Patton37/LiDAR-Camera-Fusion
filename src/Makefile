CC = g++ 
CFLAGS = -std=c++11 -Wall 
EXEC = myprogram
OBJS = main.o clustering.o 
vpath %.h ../experiment1/include
vpath %.cpp ../experiment1/src

${EXEC}: ${OBJS} 
	${CC} -o ${EXEC} ${CFLAGS} ${OBJS} 

main.o: main.cpp clustering.h
	${CC} ${CFLAGS} -c main.cpp $< ${CFLAGS} 

clustering.o: clustering.cpp clustering.h  
	${CC} ${CFLAGS} -c clustering.cpp $< ${CFLAGS} 

clean: 
	rm -f ${EXEC} ${OBJS}
	@echo "All clean!" 

      
