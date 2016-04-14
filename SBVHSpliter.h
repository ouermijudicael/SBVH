#include <iostream>
#include <vector>
#include <primitive.h>
#include <SBVHNode.h>


using namespace std;


#ifndef SBVHSPLITER_H
#define SBVHSPLITER_H

//**************************************************************
// sort_dim -> dimensions sorted
// split_location -> where to split on the sort_dim
// sahPl -> sah of left 
// sahPr -> sah of right
// nP_   -> number of primitive (Trace or Line)
// IDs_pl ->primitives ids to the left
//**************************************************************
	
struct SBVHObjectSplit
{
    int 		sort_dim;
    int 		Pl, Pr;
    float 		split_location;
    float		Cost;
    bool 		small_to_split;

    void            	print()
    {
           cerr <<"to small to be split  = " <<small_to_split  << endl;
           cerr <<"sort_dim = " <<sort_dim << endl;
           cerr <<"split_locatioin = " <<split_location << endl;
           cerr << "Cost =" << Cost << endl;
 	   cerr << "Pl = " << Pl << "Pr =" << Pr << endl; 
    }
};

class SBVHSpliter: public SBVHData
{
  public:
    //SBVHNode * tree;
    vector<SBVHNode> tree;
    int tree_size;
    // many call to build tree
    SBVHSpliter();
    ~SBVHSpliter();

    //calculates surface area heuristic 
    float 		calculateSAH(float *);

    //
    void 		append(vector<int>&, int);
    void 		append(vector<Segment>&, Segment &);

    // update bbox and saves it in the first input
    void 		updateBbox(float * , float *);

    // updates all the ancesters 
    void		updateAncesters(SBVHNode &, Segment, int);

    // different ways of finding best split location
    void		findObjectSplit(SBVHObjectSplit &, SBVHNode &);
    void		findObjectSplit1(SBVHObjectSplit &, SBVHNode &);
    void		findObjectSplitMedian(SBVHObjectSplit &, SBVHNode &);
    void		findBestSplit(SBVHObjectSplit &, SBVHNode &);
    void		findBestSpatialSplit(SBVHObjectSplit &, SBVHNode &);
    void 		findBestObjectSplit(SBVHObjectSplit &, SBVHNode &);

    // perform the split base of best locatioin found.
    void 		performObjectSplit(SBVHObjectSplit &, SBVHNode &, SBVHNode &, SBVHNode &);

    //int 		getSplitDim();
    //float		getSplitLocation();

    // Builds a the tree
    void		buildTree(SBVHNode &, int);
    void		build();

    // Find all tracers with in a radius
    void 		findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack);
    void	 		findNode(float *, float, vector<int> & );
    void	 		traditionalFind(float *, float, vector<int> &);

    // checks if a point is in the node
    bool 		isInNode(int , float *, float);

    // to keep the leaves with in threshold volume
    bool		lessThanMinimumVolume(float *);



    // checks the line cross split volume.
    bool 		isOverlaped( float *, int, float);
    bool 		isOverlaped( float *, int, float *);
    bool 		isOverlaped1( float *, int, float *);
    bool 		isOverlaped( float *, float *);

    void 		test();

    // not implemented yet
    //void 		performeSpacialSplit();

    // to calculate the tree size
    void                getTreeSizeInByte(SBVHNode &, int & );

    // to get empty nodes 
    void 		getEmptyNodes(SBVHNode &);


    // To test tite bounds
    bool 		testBounds(SBVHNode & );
};

#endif
