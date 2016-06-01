#include <primitive.h>
#include <vector>
#include <tracer.h>

#ifndef SBVHNODE_H
#define SBVHNODE_H

using namespace std;

class SBVHNode
{
    //number of primitives in node
    int        		numprimitives;
    
    // cotains the bounding box
    float		bbox[8];

    // location of left child
    int left_child;

    //location of right child
    int right_child;

    // location of parent
    int parent;

    // number of times the node has been been visited
    int visits;

    // dimension of the best split
    int split_dim;

    // where the best split is
    float split_location;

  public:
    void Print(ostream &, int, int, std::vector<SBVHNode> &);
    int getSizeOfTree(std::vector<SBVHNode> &);
    int getSizeOfData(std::vector<Segment> &);


    //cotains indeces of different primitives
    //change later to array
    vector<int>	segment_indeces;
    //int * segment_indeces;

    // shallow copy of Node
    // change later to array
    vector<Segment> node_spec;
    

    SBVHNode();
    //SBVHNode(SBVHNode &);
    ~SBVHNode();

    // this function sets parent and children
    // int 0 -> has parent and children
    void 	setLeftChild(int m);
    void 	setRightChild(int m);
    void	setParent(int m);

    //sets dimension of the split
    void       setSplitDim(int );

    // sets location of the split
    void       setSplitLocation(float);


    // sets the bounding box of the node
    void       setBbox(float *);
    
    // sets the primtives index
    void       setPrimitivesIndex(vector<int> &, int );
    void       setPrimitivesIndex(vector<Segment>&, int  );

    // append to node spec
    void       append(int);
    void       append(Segment);

    // set the visits to -1
    void       setVisit();
 
    // returns location of left child
    int           		getLeftChild();

    // check if the boox is small
    bool			isSmall(float s);
    // returns location of right child
    int           		getRightChild();

    // returns updated the number of times the node was visited
    int				getParent();

    // returns updated the number of times the node was visited
    int				getVisit();

    // returns bouding box of node
    float *    			getBbox();

    // returns heuristic surcface
    float          		getSAH();

    // returns the number of primitives
    int        			getNumPrimitives();

    // returns the dim of the split
    int 			getSplitDim();

    // returns spit location
    float			getSplitLocation();

    // Asignment operator
    SBVHNode & operator = (const SBVHNode & node);

    // return the adress of the primitives indeces
    vector<int>&	    	getPrimitivesIndex();

    // return the adrees of the shallow copy of node
    vector<Segment>&		getPrimitivesSpec();

    // delete the shallow copy
    void			clear();

    // Returns the Size of the node in Byte
    int 			getSizeInByte();
    // prints a summary
    void       			print();

    // test
    void			test();
};
#endif
