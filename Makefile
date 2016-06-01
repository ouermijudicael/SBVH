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
	g++ -O3$(OBJ_BVH) -o bvh
	echo ./bvh

sbvh: $(OBJ_SBVH)
	g++ $(OBJ_SBVH) -o sbvh
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

moveVH100:
	mv SKDTfile SKDTfileVH100
	mv KDTfile KDTfileVH100
	mv BVHfile BVHfileVH100
	mv SBVHfile SBVHfileVH100
	mv SKDTfileVH100 test_result/
	mv KDTfileVH100 test_result/
	mv BVHfileVH100 test_result/
	mv SBVHfileVH100 test_result/
moveVH1000:
	mv SKDTfile SKDTfileVH1000
	mv KDTfile KDTfileVH1000
	mv BVHfile BVHfileVH1000
	mv SBVHfile SBVHfileVH1000
	mv SKDTfileVH1000 test_result/
	mv KDTfileVH1000 test_result/
	mv BVHfileVH1000 test_result/
	mv SBVHfileVH100 test_result/

moveVH10000:
	mv SKDTfile SKDTfileVH10000
	mv KDTfile KDTfileVH10000
	mv BVHfile BVHfileVH10000
	mv SBVHfile SBVHfileVH10000
	mv SKDTfileVH10000 test_result/
	mv KDTfileVH10000 test_result/
	mv BVHfileVH10000 test_result/
	mv SBVHfileVH10000 test_result/
moveVH100000:
	mv SKDTfile SKDTfileVH100000
	mv KDTfile KDTfileVH100000
	mv BVHfile BVHfileVH100000
	mv SBVHfile SBVHfileVH100000
	mv SKDTfileVH100000 test_result/
	mv KDTfileVH100000 test_result/
	mv BVHfileVH100000 test_result/
	mv SBVHfileVH100000 test_result/

moveSM100:
	mv SKDTfile SKDTfileSM100
	mv KDTfile KDTfileSM100
	mv BVHfile BVHfileSM100
	mv SBVHfile SBVHfileSM100
	mv SKDTfileSM100 test_result/
	mv KDTfileSM100 test_result/
	mv BVHfileSM100 test_result/
	mv SBVHfileSM100 test_result/
moveSM1000:
	mv SKDTfile SKDTfileSM1000
	mv KDTfile KDTfileSM1000
	mv BVHfile BVHfileVH1000
	mv SBVHfile SBVHfileSM1000
	mv SKDTfileSM1000 test_result/
	mv KDTfileSM1000 test_result/
	mv BVHfileSM1000 test_result/
	mv SBVHfileSM100 test_result/

moveSM10000:
	mv SKDTfile SKDTfileSM10000
	mv KDTfile KDTfileSM10000
	mv BVHfile BVHfileSM10000
	mv SBVHfile SBVHfileSM10000
	mv SKDTfileSM10000 test_result/
	mv KDTfileSM10000 test_result/
	mv BVHfileSM10000 test_result/
	mv SBVHfileSM10000 test_result/
moveSM100000:
	mv SKDTfile SKDTfileSM100000
	mv KDTfile KDTfileSM100000
	mv BVHfile BVHfileSM100000
	mv SBVHfile SBVHfileSM100000
	mv SKDTfileSM100000 test_result/
	mv KDTfileSM100000 test_result/
	mv BVHfileSM100000 test_result/
	mv SBVHfileSM100000 test_result/

movePM100:
	mv SKDTfile SKDTfilePM100
	mv KDTfile KDTfilePM100
	mv BVHfile BVHfilePM100
	mv SBVHfile SBVHfilePM100
	mv SKDTfilePM100 test_result/
	mv KDTfilePM100 test_result/
	mv BVHfilePM100 test_result/
	mv SBVHfilePM100 test_result/
movePM1000:
	mv SKDTfile SKDTfilePM1000
	mv KDTfile KDTfilePM1000
	mv BVHfile BVHfileVH1000
	mv SBVHfile SBVHfilePM1000
	mv SKDTfilePM1000 test_result/
	mv KDTfilePM1000 test_result/
	mv BVHfilePM1000 test_result/
	mv SBVHfilePM100 test_result/

movePM10000:
	mv SKDTfile SKDTfilePM10000
	mv KDTfile KDTfilePM10000
	mv BVHfile BVHfilePM10000
	mv SBVHfile SBVHfilePM10000
	mv SKDTfilePM10000 test_result/
	mv KDTfilePM10000 test_result/
	mv BVHfilePM10000 test_result/
	mv SBVHfilePM10000 test_result/
movePM100000:
	mv SKDTfile SKDTfilePM100000
	mv KDTfile KDTfilePM100000
	mv BVHfile BVHfilePM100000
	mv SBVHfile SBVHfilePM100000
	mv SKDTfilePM100000 test_result/
	mv KDTfilePM100000 test_result/
	mv BVHfilePM100000 test_result/
	mv SBVHfilePM100000 test_result/





.cpp.o: $<
	g++ -g -O3 -I. -c $<


clean:
	rm -rf *.o exec bvh sbvh tracers test BVHfile SBVHfile KDTfile SKDTfile


