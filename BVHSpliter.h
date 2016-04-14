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

struct ObjectSplit
    {
    int 		sort_dim;
    int 		Pl;
    int 		Pr;
    float		split_location;
    float	  	Cost;
    bool		small_to_split;

    void            print()
    {
           cerr <<"sort_dim = " <<sort_dim << endl;
           cerr <<"split_locatioin = " <<split_location << endl;
           //cerr << "Cost =" << Cost << endl;
    }
};

//class BVHSpliter: public SBVHData, public SBVHTree
class BVHSpliter: public SBVHData
{
  protected:
    //float     cost;
              
  public:
    vector<SBVHNode> tree;
    //SBVHNode * tree;
    
    // to hold the tree size
    int tree_size;
    // many call to build tree
    BVHSpliter();
    ~BVHSpliter();

    //calculates surface area heuristic 
    float 		calculateSAH(float *);

    //
    void 		append(vector<int>&, int);
    void 		append(vector<Segment>&, Segment &);

    // update bbox and saves it in the first input
    void 		updateBbox(float * , float *);

    // used when there is a overlap among the primitives
    void		findObjectSplit(ObjectSplit &, SBVHNode &);
    void                findObjectSplit1(ObjectSplit &, SBVHNode &);
    void                findObjectSplitMedian(ObjectSplit &, SBVHNode &);
    void                findBestSplit(ObjectSplit &, SBVHNode &);
    void                findBestSpatialSplit(ObjectSplit &, SBVHNode &);
    void 		findBestObjectSplit(ObjectSplit &, SBVHNode &);



    // perform the split base of best locatioin found.
    void 		performObjectSplit(ObjectSplit &, SBVHNode &, SBVHNode &, SBVHNode &);
    void 		performeSpacialSplit();

    //int 		getSplitDim();
    //float		getSplitLocation();

    // Builds a the tree
    void		buildTree(SBVHNode &, int);
    void		build();

    // Find all tracers with in a radius
    void 		findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack);
    void	 	findNode(float *, float, vector<int> & );
    void	 	traditionalFind(float *, float, vector<int> &);

    // checks if a point is in the node
    bool 		isInNode(int , float *, float);

    // to checnk if segement overlap a specific bounding box
    bool                isOverlaped(float *  bb, int dim, float * s);
    bool                isOverlaped(float *  bb, float * s);
    bool                isOverlaped(float *  bb, int dim, float  s);

    void 		test();

    // not implemented yet

    // to calculate the tree size
    void                getTreeSizeInByte(SBVHNode &, int & );
};

#endif
