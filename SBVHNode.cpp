#include <primitive.h>
#include <SBVHNode.h>
#include <tracer.h>

#include <cstdlib>
#include <iostream>
#include <vector>


using namespace std;


//****************************************
// This constructor is called when it    *
// is the root                           *
//****************************************
SBVHNode::SBVHNode()
{
    left_child = -10;
    right_child = -10;
    parent = -10;
    visits = -1;
    split_location = -10;
    split_dim = -10;
    numprimitives = -10;
    segment_indeces.resize(0);
}

int SBVHNode::getSizeOfTree(std::vector<SBVHNode> & t)
{
    int result = sizeof(t);
    for(int i=0; i<t.size(); i++)
    {
       result = result + sizeof(float)*8 + sizeof(t[i])  + sizeof(int)*t[i].getNumPrimitives();
    }
    return result;
}

int SBVHNode::getSizeOfData(std::vector<Segment> & d)
{

    int result = sizeof(d);
    for(int i=0; i<d.size(); i++)
    {
	result = result + sizeof(d[i]) + sizeof(float)*16;
    }
    return result;

}

void SBVHNode::Print(ostream &out, int level, int node, std::vector<SBVHNode> &list)
{
    out << "-----Node " << node << "------" << endl;
    //out << "I am node " << node << endl;
    //out << "I am at level " << level << endl;
    //out << "My split dimension is " << split_dim << endl;
    //out << "My split location is " << split_location << endl;
    out << "I  have " << getNumPrimitives() << " primitives"  << endl;
    //out << "Left child for " << node << " is " << left_child << endl;
    //out << "Right child for " << node << " is " << right_child << endl;
    out << "Node bbox = " << bbox[0] <<"__" << bbox[1] << ", " << 
     			     bbox[2] <<"__" << bbox[3] << ", " <<
    			     bbox[4] <<"__" << bbox[5] << ", " <<
    			     bbox[6] <<"__" << bbox[7] <<")" << endl; 
    //for(int i=0; i<getNumPrimitives(); i++)
    //{
    //	out << "segment" << i<< " id = " << getPrimitivesIndex()[i] << endl;
    //}
    if (left_child >= 0)
        list[left_child].Print(out, level+1, left_child, list);
    if (right_child >= 0)
        list[right_child].Print(out, level+1, right_child, list);
    out << "---End Node " << node << "----" << endl;
}
/*
SBVHNode::SBVHNode (SBVHNode & node)
{
    numprimitives = node.numprimitives;
    for(int i=0; i<4; i++)
        bbox[i] = node.getBbox()[i];
    left_child = node.getLeftChild();
    right_child = node.getRightChild();
    parent = node.getParent();
    visits = node.getVisit() - 1;
    split_dim = node.getSplitDim();
    split_location = node.getSplitLocation(); 
    segment_indeces.resize(numprimitives);
    for(int i=0; i<numprimitives; i++)
    {
        segment_indeces[i] = node.segment_indeces[i];
	node_spec[i] = node.node_spec[i];
    }
}
*/
SBVHNode::~SBVHNode()
{}

void       		SBVHNode::setLeftChild(int m)		{ left_child = m; }
void       		SBVHNode::setRightChild(int m)		{ right_child = m; }
void       		SBVHNode::setParent(int m)		{ parent = m; }

//***********************************************************************************
// Set bounding box
//***********************************************************************************
void       	SBVHNode::setBbox(float * b)
{
    for(int i=0; i<8; i++)
    	bbox[i] = b[i];
}


//**********************************************************************************
// copy primitives into Nodes 
//**********************************************************************************
void  	   		SBVHNode::setPrimitivesIndex(vector<int>& p, int  m)
{
    segment_indeces.resize(m);
    //cerr << "In setPrimitivesIndex size = " << segment_indeces.size() << endl;
    for(int i=0; i<m; i++)
    {
	//cerr << "i = " << i << " p[i] = " << p[i]<< endl;
  	segment_indeces[i] = p[i]; // p[i];
  	//segment_indeces.push_back(p[i]);

    }
    //copy(p.begin(), p.begin()+m, back_inserter( segment_indeces) );
    //cerr << "segment idecies size =" << segment_indeces.size()<< endl;
}

void  	   		SBVHNode::setPrimitivesIndex(vector<Segment>& p, int  m)
{
    //node_spec.clear();
    node_spec.resize(m);
    numprimitives = m;
    for(int i=0; i<m; i++)
  	node_spec[i] = p[i];
    //copy(p.begin(), p.begin()+m, back_inserter(node_spec) );
}


void	   		SBVHNode::setSplitDim(int dim)			{split_dim = dim; }

void	   		SBVHNode::setVisit()				{visits = -1; }

void	   		SBVHNode::setSplitLocation(float location)	{split_location = location; }

void 	   		SBVHNode::append(int idx)			
{
	//cerr << "appending idx =" <<idx << endl;
	//cerr << "appending idx =" <<segment_indeces[0] << endl;
	//segment_indeces.resize(numprimitives+1);
	//segment_indeces[numprimitives] = idx;
	segment_indeces.push_back(idx);
	numprimitives++;
}

SBVHNode & 		SBVHNode::operator= (const SBVHNode & node)
{
    numprimitives = node.numprimitives;
    for(int i=0; i<4; i++)
        bbox[i] = node.bbox[i];
    left_child = node.left_child;
    right_child = node.right_child;
    parent = node.parent;
    visits = node.visits;
    split_dim = node.split_dim;
    split_location = node.split_location; 
    segment_indeces.resize(numprimitives);
    for(int i=0; i<numprimitives; i++)
    {
        segment_indeces[i] = node.segment_indeces[i];
	node_spec[i] = node.node_spec[i];
    }
}

bool			SBVHNode::isSmall(float s)
{
     bool result = false;
     if(abs(bbox[1]-bbox[0]) < s || abs(bbox[3] - bbox[2]) < s ||
	abs(bbox[5]-bbox[4]) < s || abs(bbox[7] - bbox[6]) < s )
	result = true;
     return result;
}

void 	   		SBVHNode::append(Segment s)			{node_spec.push_back(s);}

int	       		SBVHNode::getLeftChild()		{ return left_child; }
int 	       		SBVHNode::getRightChild()		{ return right_child; }
int	       		SBVHNode::getParent()			{ return parent; }
int	      		SBVHNode::getVisit()			{ return visits++; }

float *			SBVHNode::getBbox()			{ return bbox; }
float          		SBVHNode::getSAH()			
{
	float sah = 0; 
	sah = (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[7] - bbox[6]) +
	      (bbox[1] - bbox[0]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) +
	      (bbox[3] - bbox[2]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) +
	      (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[5] - bbox[4]) ;
	if(sah == 0.00 && abs(bbox[1]-bbox[0]) <= 0.000001)
        {

	     sah = 2*((bbox[3] -bbox[2]) * (bbox[7] - bbox[6]) +
	           (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) +
	           (bbox[3] -bbox[2]) * (bbox[5] - bbox[4])) ;
	}
	else if(sah ==0 && abs(bbox[3]-bbox[2]) <= 0.000001)
        {
	    sah = 2*( (bbox[1] - bbox[0]) * (bbox[7] - bbox[6]) +
	          (bbox[5] - bbox[4]) * (bbox[7] - bbox[6]) +
	          (bbox[1] - bbox[0]) * (bbox[5] - bbox[4])) ;
	}
	else if(sah ==0 && abs(bbox[5]-bbox[4]) <= 0.000001)
	{

	      sah = 2*( (bbox[1] - bbox[0]) * (bbox[7] - bbox[6]) +
	            (bbox[3] - bbox[2]) * (bbox[7] - bbox[6]) +
	            (bbox[1] - bbox[0]) * (bbox[3] - bbox[2]) ) ;
	}
	else if(sah ==0 && abs(bbox[7]-bbox[6]) <= 0.000001)
	{
	
	    sah = 2 *( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2])  +
	          (bbox[1] - bbox[0]) * (bbox[5] -bbox[4]) +
	          (bbox[3] - bbox[2]) * (bbox[5] -bbox[4]) ) ;
	}

	return sah;
}
vector<int>&     	SBVHNode::getPrimitivesIndex()		{ return segment_indeces; } 
vector<Segment>&    	SBVHNode::getPrimitivesSpec()		{ return node_spec; } 
int 			SBVHNode::getNumPrimitives()		{ return segment_indeces.size();}
int	   		SBVHNode::getSplitDim()			{ return split_dim; }
float	   		SBVHNode::getSplitLocation()		{ return split_location; }

void			SBVHNode::clear() 			{ node_spec.clear();}

void 			SBVHNode::test()
{
    int siz = segment_indeces.size();
    for(int i=0; i<siz; i++)
    {
    
    }
}

// *********************************************
// to print summary                            *
//**********************************************
void       	SBVHNode::print()
{
    cerr << "number of primitives =" << getNumPrimitives()<< endl;
    cerr << "Node bbox =( " << bbox[0] << "__"<< bbox[1] << ", "
    			  << bbox[2] << "__"<< bbox[3] << ", "
    			  << bbox[4] << "__"<< bbox[5] << ", "
       			  << bbox[6] << "__"<< bbox[7] << ") " << endl;
    cerr << "left Child = " << getLeftChild() << endl;
    cerr << "right Child = " << getRightChild() << endl;
    cerr << "parent  = " << getParent() << endl;
    //cout <<" indeces = " ;
    //for(int i=0; i<getNumPrimitives(); i++)
//	cout << segment_indeces[i] << ", ";
  //  cout <<endl;

}

/*
//
int 			SBVHNode::getSizeInByte()
{
    int result = sizeof(int)*(7 + primitive_index.size()) + sizeof(primitive_index) + sizeof(double)*2;
    for(int i=0; i<node_spec.size(); i++)
    {
	result = result + sizeof(node_spec[i]) + node_spec[i].getSizeInByte();
    }
    //cerr << "node spec size = " << node_spec.size() << endl;
    return result;
}


*/
