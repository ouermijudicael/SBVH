#include <vector>
#include <tracer.h>

#ifndef PRIMITIVE_H
#define PRIMITIVE_H

using namespace std;

class SBVHData
{
  protected:
    //int n;
    // bbox size 8
    float bbox[8];
  public:
    // holds  the data (primitives)
    vector<Segment> 	data;

    //data_spec and data_idx must be clear
    // once copied in the root
    vector<int>		data_idx;
    
    SBVHData();
    ~SBVHData();

      //functions

      // adds a tracer to the data
      void              addSegment(Segment &);

      // add a shallow coppy of data
      //void              addTracerSpec(tracer_spec &);

      // Updates bbox and saves it in first input
      void  		updateBbox(float *, float *);

      // read file with tracers
      void 		read(const char * filename);

      // this function parses a string into tracer
      void 		split(string input);
      
      // returns the  the data size num of primitives
      int  		getDataSize();

      // returns the indeces of the data
      vector<int>&	getDataIndex();

      // returns the bbox of the data
      float *  		getDataBbox();

      // Returns size of data in byte
      int 		getSizeInByte();

      void 		test();

      void      	print();
};

#endif
