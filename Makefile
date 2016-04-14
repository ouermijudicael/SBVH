SRC_SBVH = tracer.cpp mainSBVH.cpp primitive.cpp SBVHnode.cpp SBVHSpliter.cpp SKDTREESpliter.cpp 
OBJ_SBVH = tracer.o mainSBVH.o primitive.o SBVHNode.o SBVHSpliter.o SKDTREESpliter.o

SRC_BVH  = tracer.cpp mainBVH.cpp primitive.cpp SBVHnode.cpp BVHSpliter.cpp KDTREESpliter.cpp 
OBJ_BVH  = tracer.o mainBVH.o primitive.o SBVHNode.o BVHSpliter.o KDTREESpliter.o

SRC_NEW = tracer.cpp  primitive.cpp SBVHNode.cpp SBVHSpliter.cpp BVHSpliter.cpp SKDTREESpliter.ccp main.cpp KDTREESpliter.cpp
OBJ_NEW = tracer.o primitive.o SBVHNode.o SBVHSpliter.o BVHSpliter.o main.o SKDTREESpliter.o KDTREESpliter.o

ABC_SRC = createABCtracers.cpp
ABC_OBJ = createABCtracers.o

test: $(OBJ_NEW)
	g++ $(OBJ_NEW) -g  -o  test
	echo ./test
bvh: $(OBJ_BVH)
	g++ -O3 $(OBJ_BVH) -o bvh
	echo ./bvh

sbvh: $(OBJ_SBVH)
	g++ -O3 $(OBJ_SBVH) -o sbvh
	echo ./sbvh

tracers: $(ABC_OBJ)
	g++ $(ABC_OBJ) -o tracers
	echo ./tracers

measure_bvh:
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh
	./bvh

measure_sbvh:
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
	./sbvh
.cpp.o: $<
	g++ -g -I. -c $<


clean:
	rm -rf *.o exec bvh sbvh tracers test


