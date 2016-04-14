#include <iostream>
#include <vector>
#include <primitive.h>
#include <SBVHNode.h>


using namespace std;


#ifndef KDTREESpliter_H
#define KDTREESpliter_H


//**************************************************************
// sort_dim -> dimensions sorted
// split_location -> where to split on the sort_dim
// sahPl -> sah of left 
// sahPr -> sah of right
// nP_   -> number of primitive (Trace or Line)
// IDs_pl ->primitives ids to the left
//**************************************************************
struct SKDTreeObjectSplit
    {
    int 		sort_dim;
    float 		split_location;
    float  		Cost;
    int 		Pl;
    int 		Pr;
    bool		small_to_split;

    void            print()
    {
           cerr <<"sort_dim = " <<sort_dim << endl;
           cerr << "*****PL*****" << endl;
           cerr <<"split_locatioin = " <<split_location << endl;
    }
};

class SKDTREESpliter: public SBVHData
{
  protected:
    float     cost;
              
  public:
    vector<SBVHNode> tree;
    int tree_size;
    // many call to build tree
    SKDTREESpliter();
    ~SKDTREESpliter();

    //calculates surface area heuristic 
    float 		calculateSAH(float *);

    //
    void 		append(vector<int>&, int);
    void 		append(vector<Segment>&, Segment &);
    
    // to update parents when split occurs
    void 		updateAncesters(SBVHNode &, Segment, int);

    // update bbox and saves it in the first input
    void 		updateBbox(float *, float *);

    // used when there is a overlap among the primitives
    void		findBestSplit(SKDTreeObjectSplit &, SBVHNode &);
    void		findObjectSplit1(SKDTreeObjectSplit &, SBVHNode &);
    void		findObjectSplitMedian(SKDTreeObjectSplit &, SBVHNode &);
    void 		findBestSpatialSplit(SKDTreeObjectSplit &, SBVHNode &);


    // perform the split base of best locatioin found.
    void 		performSplit(SKDTreeObjectSplit &, SBVHNode &, SBVHNode &, SBVHNode &);

    //int 		getSplitDim();
    //double		getSplitLocation();

    // Builds a the tree
    void		buildTree(SBVHNode &, int);
    void		build();

    // Find all tracers with in a radius
    void 	        findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack);
    void	 	findNode(float *, float, vector<int> & );
    void	 	traditionalFind(float *, float, vector<int> &);

    // checks if a point is in the node
    bool 		isInNode(int , float *, float);

    // threshold volume
    bool 		lessThanMinimumVolume(float *);

    //cehcks if 2 bbox overlap
    bool		isOverlaped(float *, int, float);
    bool		isOverlaped(float *, int, float *);
    bool		isOverlaped(float *, float *);
    bool		isOverlaped1(float *, int, float *);

    // to calculate the tree size
    void                getTreeSizeInByte(SBVHNode &, int & );

    void 		test();

};

#endif
