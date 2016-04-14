#include <primitive.h>
#include <tracer.h>
#include <iostream>
#include <cmath>
# include <vector>
# include <string>
# include <stdio.h>
# include <stdlib.h>
# include <fstream>
# include <sstream>

double debug = 0;

using namespace std;
SBVHData::SBVHData()
{
    for(int i=0; i<8; i++)
	bbox[i] = -10000;
}

SBVHData::~SBVHData()
{
}

//**************************************************************
// push tracer in the data
//*************************************************************
void SBVHData:: addSegment(Segment & t)
{
    data.push_back(t);
    
}

//*****************************************************************
// bb1 --> bbox1
// bb2 --> bbox2
// This function takes 2 bbox and updates the first to a new bbox
//*****************************************************************
void SBVHData::updateBbox(float * bb1, float * bb2)
{
    if(bb1[0] == -10000)
    {
 	for(int i=0; i< 4; i++)
    	{
	    bb1[i*2] = bb2[i*2];
	    bb1[i*2 + 1] = bb2[i*2 + 1];
	}
    }
    else
    {
        for(int i=0; i< 4; i++)
        {
       	    if(bb2[i*2] < bb1[i*2])
	        bb1[i*2] = bb2[i*2];
            if(bb2[i*2 + 1] > bb1[i*2 + 1])
	        bb1[i*2 + 1] = bb2[i*2 + 1];
        }
    }
}


//********************************************************************
// a parser to parse a line obtained from input file
//**********************************************************************
void SBVHData::split(string input) 
{
    //cerr << "begin of split" << endl;
    if( input != "" )
    {
	float pt0[4];
	float pt1[4];
	stringstream s(input);

	string result[4];
	string tracerString;
	//	Tracer thisTrace;
	s >> tracerString;
	int tracer_id = atoi(tracerString.c_str());
	//cerr << "tracer id = " << tracer_id << endl;
	tracerString = "";
	s >> tracerString;
	int num_of_point_in_tracer = atoi(tracerString.c_str());
	//cerr << "tracer number of points = " << num_of_point_in_tracer << endl;

	// gets the first point
	for(int j=0; j<4; j++)
	{
		string tmp;
		s >> tmp;
		result[j] = tmp;
	}

	pt0[0]  = atof(result[0].c_str());
	pt0[1]  = atof(result[1].c_str());
	pt0[2]  = atof(result[2].c_str());
	pt0[3]  = atof(result[3].c_str());
	//while(!s.eof())
	for(int i=1; i < atoi(tracerString.c_str()); i++)
	{
	    for(int j=0; j<4; j++)
	    {
		string tmp;
		s >> tmp;
		result[j] = tmp;
	    }

	    pt1[0]  = atof(result[0].c_str());
	    pt1[1]  = atof(result[1].c_str());
	    pt1[2]  = atof(result[2].c_str());
	    pt1[3]  = atof(result[3].c_str());
	    //cerr << pt1[1] << endl;
	    Segment seg;
	    for(int k =0; k < 4; k++)
	    {
		seg.pt0[k] = pt0[k]; 
		seg.pt1[k] = pt1[k];

		//update pt0
		pt0[k] = pt1[k];
	    }

            seg.id = data.size();
            seg.tracer_id = tracer_id;
	    seg.setBbox();
	    data_idx.push_back(data.size());
	    addSegment(seg);
	    updateBbox(bbox, seg.bbox);
	}
    }
}

//*****************************************************************
// reads file 
//*****************************************************************
void SBVHData::read(const char * filename)
{
    //cerr << "Begining read" << endl;

    // set index of tracers
    ifstream inFile(filename);
    string strOneLine;
    getline(inFile, strOneLine);
    cout << strOneLine << endl;

    while (inFile)
    //for(int i =0; i < 10; i++)
    {
      
       //cerr << "Begining of While" << endl;
       getline(inFile, strOneLine);

       split(strOneLine);
       //cerr << "Done parsing line :" << endl;
        //cerr << "data size = "<<data.size() << endl;
       // initialise d_bb
    }
    inFile.close();
    if(debug == 1)
	cerr << "leaving read method " << endl;

}

void     SBVHData::print()
{
    cerr << "Data Size =" << getDataSize() << endl;
    cerr << "Data bbox =" << getDataBbox()[0] <<"-" <<getDataBbox()[1] << ","
                          << getDataBbox()[2] <<"-" <<getDataBbox()[3] << ","
                          << getDataBbox()[4] <<"-" <<getDataBbox()[5] << ","
                          << getDataBbox()[6] <<"-" <<getDataBbox()[7] << endl;
}

int      	SBVHData::getDataSize()		{return data.size();}

float * 	SBVHData::getDataBbox()		{return bbox; }

int 	SBVHData::getSizeInByte()
{
    int result = 0;
    return result = ( sizeof(float)+sizeof(int)+ sizeof(data[0]) ) * data.size();
}

vector<int>&     SBVHData::getDataIndex()	{return data_idx;}

void 		SBVHData::test()
{
    read("ABCtracer");
    int size = getSizeInByte();
    cerr << "Size =" << size << endl;
    print();
}

