#include <SKDTREESpliter.h>
#include <SBVHNode.h>
#include <primitive.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

#define SKDTREE_MIN_NUM_PRIMITIVES 200
// for debugging purposes
int SKDTREEdebuger = 0;
int  toy_skdt = 1;
int info_skdt = 0;
int initial_data_size_skdt = 0;
int initial_num_of_primitives_skdt =0;


using namespace std;

SKDTREESpliter::SKDTREESpliter()
{
    tree_size = -10;
    tree.resize(0);
}

SKDTREESpliter::~SKDTREESpliter()
{
}

// *****************************************
// This function calculates SAH
//******************************************
float  SKDTREESpliter::calculateSAH(float * bbox)
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

//*****************************************************************
// bb1 --> bbox1
// bb2 --> bbox2
// This function takes 2 bbox and updates the first to a new bbox
//*****************************************************************
void SKDTREESpliter::updateBbox(float * bb1, float * bb2)
{
    if(bb1[0] == -10000 || bb1[1] == -10000)
    {
	for(int i=0; i<4; i++)
        {
	    bb1[i*2] = bb2[i*2];
	    bb1[i*2 + 1] = bb2[i*2 + 1];
    	}
    }
    else
    {
        for(int i=0; i<4; i++)
        {
       	    if(bb2[i*2 + 0] < bb1[i*2 + 0])
	        bb1[i*2 + 0] = bb2[i*2 + 0];
	    if(bb2[i*2 + 1] > bb1[i*2 + 1])
	        bb1[i*2 + 1] = bb2[i*2 + 1];
    	}
    }

}

//********************************************************************
// This function appends an id i to a list of object location
//*******************************************************************
void SKDTREESpliter::append(vector<int>& arr, int id)
{
    arr.push_back(id);
}

void SKDTREESpliter::append(vector<Segment>& arr, Segment & id)
{
    arr.push_back(id);
}

//*********************************************************************
// This updates all the ancesters after a split is performed
// to make sure node conserves there initegraty
//**********************************************************************
void SKDTREESpliter::updateAncesters(SBVHNode & node, Segment s, int idx)
{
    int parent_idx = node.getParent();
    node.append(idx);
    node.append(s);
    //cerr << "idx =" << idx << endl; 
    while(parent_idx > -1)
    {
	//tree[parent_idx].append(s);
	tree[parent_idx].append(idx);
	parent_idx = tree[parent_idx].getParent();
    }
}

bool		SKDTREESpliter::lessThanMinimumVolume(float * bb)
{
    bool result = false;

    if( (abs(bb[1]-bb[0]) < .002) && (abs(bb[3]-bb[2]) < .002) && (abs(bb[5]-bb[4]) < .002) && (abs(bb[7]-bb[6]) < .002) )
	result = true;
/*
    if(abs(bb[1]-bb[0]) < .002*2)
	result = true;
    else if(abs(bb[3]-bb[2]) < .002*2)
	result = true;
    else if(abs(bb[5]-bb[4]) < .002*2)
	result = true;
    else if(abs(bb[7]-bb[6]) < .002*2)
	result = true;
*/
    return result;
}

//************************************************************
// checks if the line crosses the volume of split (4D)
//***********************************************************
bool		SKDTREESpliter::isOverlaped(float * bb, int dim, float s)
{
    bool result = false;

    if((bb[2*dim+1] - bb[2*dim+0]) >0 && bb[2*dim+0] < s && s<bb[2*dim+1])
	result = true;
    //else if((bb[2*dim+1] - bb[2*dim+0]) <0  && bb[2*dim+1] < s && s < bb[2*dim+1])
	//result = true;
    return result;
}

bool		SKDTREESpliter::isOverlaped1(float *  bb, int dim, float * s)
{
    bool result = true;
    if( (bb[2*dim+1] - bb[2*dim+0]) >0 &&
	((bb[2*dim+0] == s[2*dim+0] && s[2*dim+1] == bb[2*dim+1]) || (bb[2*dim+1]== s[2*dim+0])|| (s[2*dim+1] == bb[2*dim+0]) ) )

	result = false;
    return result;
 
}

bool		SKDTREESpliter::isOverlaped(float *  bb, int dim, float * s)
{
    bool result = false;
    if( (bb[2*dim+1] - bb[2*dim+0]) >0 &&
	((bb[2*dim+0] < s[2*dim+0] && s[2*dim+0]<bb[2*dim+1]) || (bb[2*dim+0] < s[2*dim+1] && s[2*dim+1] < bb[2*dim+1]) ) )
	result = true;
    //else if( (bb[2*dim+1] - bb[2*dim+0]) < 0 &&
    //	((bb[2*dim+1] < s[2*dim+0] && s[2*dim+0]<bb[2*dim+0]) || (bb[2*dim+1] < s[2*dim+1] && s[2*dim+1] < bb[2*dim+0]) ) )
    //	result = true;
    return result;
}

/*
//*************************************************************
// Uses preordertraversal to claculate the size of tree
//*************************************************************
void 		SKDTREESpliter::getTreeSizeInByte(SBVHNode & root, int & result)
{
   int result_l =0;
   int result_r =0;
   if(root.getLeftChild() !=-2)
	getTreeSizeInByte(tree[root.getLeftChild()], result_l);
   if(root.getRightChild() !=-2)
	getTreeSizeInByte(tree[root.getRightChild()], result_r);
   //cerr << result_l << "------" << result_r << endl;
   result = result + result_l + result_r + sizeof(root) + root.getSizeInByte();
}
*/
//**************************************************************
// This use sah to find the best Split 
// for the data
//**************************************************************
void 	SKDTREESpliter::findBestSpatialSplit(SKDTreeObjectSplit & result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    int temp_Pl = 0; 
    int temp_Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    float left_range = 10;
    float right_range = 10;

    //float range = 0;
    result.Cost = 10000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;

    //bool right_bound = true;
    int split_case = 0;
    //iset the temp bbox
    float left_bbox[8];
    float  right_bbox[8];
    float temp_split_location = -10000000;
    float * seg_bb;

    //bool left_overlap;
    //bool right_overlap;
    float sah = -10000;
    int step = 1;
    int threshold_num = -10;
    int threshold_num_begin= -10;
    int P_span= 0;
    
    if(n_node > 1000)
	step = floor(n_node/1000);
	//step =1;
    //loop through possible dimenssion
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
      split_case = -1;
      while(split_case < 1)
      {// begining of while
        split_case++;

	// sort By axis
    	if(idx_dim == 0 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_lower_bound());
    	else if(idx_dim == 0 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_upper_bound());
        else if(idx_dim == 1 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_lower_bound());
    	else if(idx_dim == 1 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_upper_bound());
    	else if(idx_dim == 2 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_lower_bound());
    	else if(idx_dim == 2 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_upper_bound());
 	else if(idx_dim == 3 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_lower_bound());
    	else if(idx_dim == 3 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_upper_bound());
        else 
	{
	   cerr <<  "Not suppose to occur : " << endl;
	}

	//cerr << "range = " << abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) << endl;
	//range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
    	Pl = 0; 
    	Pr = 0;
        temp_Pl =0;
        for(int idx_node=0; idx_node<n_node; idx_node= idx_node +step)
	{

	    //cerr << "idx_node =" << idx_node << endl; 
       	    seg_bb = node.node_spec[idx_node].getBbox();
	    //node.node_spec[idx_node].print();

            if(split_case == 0 && seg_bb[idx_dim*2+0] != bbox[idx_dim*2+0])
            {
            	temp_split_location = seg_bb[idx_dim*2+0];
	    }
	    else if(split_case == 0 && seg_bb[idx_dim*2+0] == bbox[idx_dim*2+0])
            {
		goto label;
            }
	    else if(split_case == 1 && seg_bb[idx_dim*2+1] != bbox[idx_dim*2+1])
	    {
            	temp_split_location = seg_bb[idx_dim*2+1];
            }
	    else if(split_case == 1 && seg_bb[idx_dim*2+1] == bbox[idx_dim*2+1])
	    {
		goto label;
	    }
      	    else if(split_case == 2) // not used yet
	    {
            	temp_split_location = (seg_bb[idx_dim*2+0] + seg_bb[idx_dim*2+1]) /2;
	    }

            // split_case++;
	    //cerr << "temp split location =" << temp_split_location <<  endl;
	    //cerr << "split case =" << split_case <<  endl;
            // setting temp bounding boxes
            for(int i=0; i<4; i++)
	    {
                if(idx_dim == i)
	        {
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = temp_split_location;
		    right_bbox[i*2+0] = temp_split_location;
		    right_bbox[i*2+1] = bbox[i*2+1];
	        }
		else
		{
	            left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
             }

// Do not remove this tajo

	    if((n_node - idx_node) > 1000)
	    	threshold_num = idx_node + 1000;
	    else
	    	threshold_num = n_node;

	    if(idx_node >1000)
		threshold_num_begin =idx_node - 1000;
	    else  
		threshold_num_begin =0;
	    temp_Pl= 0;
/*
            for(int idx_tracer=threshold_num_begin; idx_tracer < idx_node; idx_tracer++)
	    {
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 
	        if(temp_split_location<=temp_bb[idx_dim*2+0])
	        {	    
	             temp_Pl++;
		}
	    }
	    Pl = idx_node - temp_Pl;

*/	    temp_Pr = 0;
            P_span = 0;
            for(int idx_tracer= threshold_num_begin; idx_tracer < threshold_num; idx_tracer++)
	    {
		//counting++;
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 

		if(temp_bb[idx_dim*2+0] < temp_split_location && temp_split_location < temp_bb[idx_dim*2+1])
		     P_span++;
	        //if(temp_split_location>=temp_bb[idx_dim*2+1])
	        //     Pl++;
	    }
       
            Pl = idx_node + P_span;
	    Pr = n_node - idx_node + P_span;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    //C = ((Pl+temp_Pl)*left_sah + (Pr+temp_Pr)*right_sah) / sah;
	    C = (Pl*left_sah + Pr*right_sah) / sah;
             
 	    left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
 	    right_range = abs(temp_split_location - bbox[idx_dim*2+1]);

            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            //cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > .0001 && right_range > .0001 && 
	     	    temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0]) //&& Pl > floor(n_node/4))
	    //if(C < result.Cost && temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0]) //&& Pl > floor(n_node/4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		result.Cost = C;
		result.Pl = Pl;
	        result.Pr = Pr;
		result.sort_dim = idx_dim;
		result.split_location = temp_split_location;
		//result.small_to_split = false;
	    }
	    else if(C = result.Cost && left_range > .0001 && right_range > .0001 && 
	     	    temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0]) //&& Pl > floor(n_node/4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
                //cerr << " abs(Pl-Pr) = "<<  abs(Pl-Pr) << endl; 
		//cerr << " abs(result.Pr - result.Pl)" << abs(result.Pr - result.Pl) << endl;
                if( abs(Pl-Pr) < abs(result.Pr - result.Pl) )
		{
                   result.Pr = Pr;
		   result.Pl = Pl;
		   result.Cost = C;
		   result.sort_dim = idx_dim;
		   result.split_location = temp_split_location;
		   //result.small_to_split = false;
	  	}
	    }
        //cerr << "Split Cost =" <<result.Cost << endl;
        //cerr << "split locatioin" << result.split_location << endl;
	//cerr << "Pl "<< result.Pl << endl;
	//cerr << "Pr "<< result.Pr << endl;

	    //Pl = 0;
	    //Pr = 0;
	label : ;
        }// end of for loop
      }// end of while
        if(result.split_location == bbox[2*result.sort_dim+0] || result.split_location == bbox[2*result.sort_dim+1])
	{
            cerr << "idx dim = " << idx_dim << endl;
	    cerr << "Pl "<< result.Pl << endl;
	    cerr << "Pr "<< result.Pr << endl;
	    cerr << "Split Cost =" <<result.Cost << endl;
	    cerr << "Split location =" <<result.split_location << endl;
	    cerr << "Split dim = " << result.sort_dim << endl;;
	    cerr << "Split case = " << split_case << endl;
 	    node.print();
	    exit(1);
	}
    }
    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0])
    {
	 cerr << "split location " << result.split_location <<" is outside of the bounds"<< endl;
	 cerr << "Not suppose to occur"<< endl;
	 cerr << "Pl "<< result.Pl << endl;
	 cerr << "Pr "<< result.Pr << endl;
	 cerr << "Split Cost =" <<result.Cost << endl;
	 cerr << "Split location =" <<result.split_location << endl;
	 cerr << "Split dim = " << result.sort_dim << endl;
	 cerr << "Split case = " << split_case << endl;
         cerr << bbox[result.sort_dim*2+0] << "__"<< bbox[result.sort_dim*2+1] << endl;
         node.print();
	 exit(1);
    }
    if(result.sort_dim < 0 || result.sort_dim > 3)
    {
	cerr << "Not suppose to occur " << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
    }

    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
}

//********************************************************
// 
//********************************************************
void SKDTREESpliter::findBestSplit(SKDTreeObjectSplit &result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;

    float sahPl, sahPr, C;
    int n =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    int dim = -1;
    float range = 0;
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	{
            dim  = idx_dim;
	    range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	}

        // maybe i need to normalize time
    }
    
/*
    // sort By axis
    if(dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    if(dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    if(dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    if(dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

*/
    //update cost and slpit location and other elements
    //float center = node.node_spec[n/2].center[dim];
    float center = (bbox[dim*2+1]+bbox[dim*2+0])/2;
    C = -10;
    //if(SBVHdebugSpliter == 1)
    //cerr << "Cost =" << C << endl;
    result.Cost = C;
    result.sort_dim = dim;
    result.split_location = center;

    node.setSplitLocation(center);
    node.setSplitDim(dim);
            
    //cerr << "Leaving findObjectSplit1" << endl;
}

void SKDTREESpliter::findObjectSplitMedian(SKDTreeObjectSplit & result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    float sahPl, sahPr, C;
    int n =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    int dim = -1;
    float range = 0;
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	{
            dim  = idx_dim;
	    range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	}
        // maybe i need to normalize time
    }
    
    if(dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    if(dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    if(dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    if(dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

    int index = floor(n/2); 

    //update cost and slpit location and other elements
    float center = node.node_spec[index].getCenter(dim);
    C = -10;
    result.Cost = C;
    result.sort_dim = dim;
    result.split_location = center;

    node.setSplitLocation(center);
    node.setSplitDim(dim);
            
    //cerr << "Leaving findObjectSplit1" << endl;
}


//************************************************************************
//
//************************************************************************
//void SKDTREESpliter::findBestSplit()
//{}


//************************************************************************
// 
//************************************************************************
void SKDTREESpliter::performSplit(SKDTreeObjectSplit & split, SBVHNode & node, SBVHNode & NL, SBVHNode & NR)
{
    //cerr << "Entering perform split" << endl;
    //split.print();
    // find the primitives that need to be slpit and split them
    int n =  node.getNumPrimitives();
    int dim = split.sort_dim; 
    float split_pos = split.split_location;
    vector<int> l_index;
    vector<Segment> l_node_spec;
    vector<int> r_index;
    vector<Segment> r_node_spec;

    float * bbox = node.getBbox();
    float l_bbox[8]; // float[8];
    float r_bbox[8]; // = new float[8];

    // initialize bbox
    for(int i =0; i<4; i++)
    {
	if(i == dim)
        {
	    l_bbox[i*2+0] = bbox[i*2+0];
	    l_bbox[i*2+1] = split_pos;
	    r_bbox[i*2+0] = split_pos;
	    r_bbox[i*2+1] = bbox[i*2+1];
	}
	else
	{
	    l_bbox[i*2+0] = bbox[i*2+0];
	    l_bbox[i*2+1] = bbox[i*2+1];
	    r_bbox[i*2+0] = bbox[i*2+0];
	    r_bbox[i*2+1] = bbox[i*2+1];
	}
    } 


    int id;
    float * line_center;
    float * bb;
    //bool overlap_left;
    //bool overlap_right;
    float center;

    for(int i =0; i<n; i++)
    {


	
	//cerr << "primitive i =" << i << endl;
        id = node.node_spec[i].id;
	//data[id].print();
	//cerr << "id =" << id << endl;;
 	center = node.node_spec[i].getCenter(dim);
	// to hold spilting results
        //new_segment.id = data.size();

        //float center = line_center[dim];
        bb = node.node_spec[i].getBbox();
/*
	
	overlap_left = isOverlaped(l_bbox,dim, bb);
        overlap_right = isOverlaped(r_bbox, dim, bb);

	if(overlap_left == true)
            overlap_left = isOverlaped1(l_bbox,dim, bb);
	if(overlap_right == true)
            overlap_right = isOverlaped1(r_bbox, dim, bb);

	bool cross_split_volume;// = isOverlaped(bb, dim, split_pos);
	if(bb[2*dim+0] < split_pos && split_pos < bb[2*dim+1])
	    cross_split_volume = true;
 	else
	    cross_split_volume = false;
	    
        
	//cerr << "overlap  left =" << overlap_left << endl;
	//cerr << "overlap  right =" << overlap_right<< endl;
	//cerr << "cross volume  =" << cross_split_volume << endl;
*/
	if(bb[2*dim+0] < split_pos && split_pos < bb[2*dim+1])
	{
	    Segment new_segment;
	    node.node_spec[i].splitSegment(split_pos, new_segment, dim);
	    data[id].splitSegment(split_pos, new_segment, dim);
            new_segment.id = data.size();
	    addSegment(new_segment);
	    //new_segment.print();
	    //data[id].print();

	    int index = data.size()-1;
		
	    //expensive
	    updateAncesters(node, new_segment, index);
            //cerr << "Moving Line for debugging" << endl;
	    if(new_segment.getCenter(dim) <= split_pos)
	    {
	        append(l_index, index);
		append(l_node_spec, new_segment);
		append(r_index, id);
		append(r_node_spec, data[id]);
	    }
	    else
	    {
	        append(r_index, index);
		append(r_node_spec, new_segment);
		append(l_index, id);
		append(l_node_spec, data[id]);
	    }
	    
	}
	else if(split_pos >= bb[2*dim+1])
    	{
           //tracer_spec temp;
           //temp.index = id;
           //copy(line_center.begin(), line_center.begin()+4, back_inserter(temp.center));
  	   append(l_index, id);
	   //cerr<< "only left" << endl;
  	   append(l_node_spec, data[id]);
	}
	else if(split_pos <= bb[2*dim+0])
	{
           //tracer_spec temp;
           //temp.index = id;
           //copy(line_center.begin(), line_center.begin()+4, back_inserter(temp.center));
  	   append(r_index, id);
	   append(r_node_spec, data[id]);
	}
	else
        {
	    cerr << "not suppose to happen: Segment not in the node " << endl;
	    cerr << "Split position = " << split_pos << endl;
	    cerr << "Split dim = " << dim << endl;
	    if(split_pos < bbox[2*dim] || split_pos> bbox[2*dim+1])
	    {
		    cerr << setprecision(10)<<"split position =" << split_pos <<" is outside of bbox" <<  endl;
		    //node.print();
	    }

	    for(int dim_test =0; dim_test<4; dim_test++)
	    {
		    if(bb[2*dim_test+0] < bbox[2*dim_test+0] || bb[2*dim_test+1] > bbox[2*dim_test+1])
		    {
			    cerr << "segment is not in the node " << endl;
			    cerr << "dim test =" << dim_test <<endl;
		    }
	    }
	    node.print();
	    node.node_spec[i].print();
	    exit(0);
	}
    } 

    ///cerr << "l_index size = " << l_index.size() << endl;
    //cerr << "r_index size = " << r_index.size() << endl;
    //l_node_spec[l_node_spec.size() -1].print();
    NL.setBbox(l_bbox);
    //NL.setSAH();
    NL.setPrimitivesIndex(l_index, l_index.size());
    //cerr << "Moving Line "<< endl;
    NL.setPrimitivesIndex(l_node_spec, l_index.size());
    //NL.setNumPrimitives(l_index.size());

    NR.setBbox(r_bbox);
    //NR.setSAH();
    NR.setPrimitivesIndex(r_index, r_index.size());
    NR.setPrimitivesIndex(r_node_spec, r_index.size());
    //NR.setNumPrimitives(r_index.size());
    
    //if(debugKDTREE == 1)
    //{
    	//cerr << "Left child Primitives =" << NL.getNumPrimitives() << endl;
    	//cerr << "Right Child Primitives="<<  NR.getNumPrimitives() << endl;
    	//cerr << "End of perform Split ..." << endl; 
    //}

}
//******************************************************************************************
// 
//******************************************************************************************
void SKDTREESpliter::buildTree(SBVHNode & node, int me)
{

     int  n = node.getNumPrimitives();
     //cerr << "In the buildTree" << endl;
     SKDTreeObjectSplit split;

     SBVHNode nodeL;
     SBVHNode nodeR;

     nodeL.setParent(me);
     nodeR.setParent(me);
     node.setLeftChild(tree.size());
     int l = tree[me].getLeftChild();
     node.setRightChild(tree.size()+1);
     int r = tree[me].getRightChild();

    //findBestSplit(split, node);
    findBestSpatialSplit(split, node);
    //findObjectSplitMedian(split, node);
    performSplit(split, node, nodeL, nodeR);
     //cerr << "After Split Operation" << endl;

    tree.push_back(nodeL);
    tree.push_back(nodeR);
    int size = tree.size();
    if(SKDTREEdebuger == 1)
    {
    cerr << "-----------------------------------------" << endl;
    cerr << "me = "<< me << "l ="<< l << "r =" << r << endl;
    cerr << "Parent Primitives =" << tree[me].getNumPrimitives() << endl; 
    cerr << "Left child Primitives =" << tree[l].getNumPrimitives() << endl;
    cerr << "Right Child Primitives="<< tree[r].getNumPrimitives() << endl;
    cerr << "Split dim =" << tree[me].getSplitDim() << endl;
    cerr << "Split locatioin =" << tree[me].getSplitLocation() << endl;
    node.print();
    }

    if(tree[l].getNumPrimitives() > SKDTREE_MIN_NUM_PRIMITIVES && tree[l].isSmall(0.001) != true)
    {
        //cerr << "conditional 1" << endl;
    	buildTree(tree[l], l);
    }
    else
    {
        tree[l].setLeftChild(-2);
        tree[l].setRightChild(-2);
    }
    if(tree[r].getNumPrimitives() >  SKDTREE_MIN_NUM_PRIMITIVES && tree[l].isSmall(0.001) != true)
    {
        //cerr << "conditional 2" << endl;
    	buildTree(tree[r], r);
    }
    else
    {
        tree[r].setLeftChild(-2);
        tree[r].setRightChild(-2);
    }

    // clean shalow copies that where use 
    // to build Tree
    tree[l].clear();
    tree[r].clear();

    //if(l==3 || r == 3)
    //{
    //    tree[l].print();
    //	tree[r].print();
    //}
    //cerr << "leaving Build tree" << endl;
    // clean the shallow copy
    // clean nodes too
}

//***********************************************************
//checks if the Node is in the Tree
//************************************************************
bool		SKDTREESpliter::isInNode(int node_idx, float * pt, float r)
{
    bool result = false;
    float *  bb = tree[node_idx].getBbox();

    if( (bb[6] <= pt[3] && pt[3] <= bb[7]) && // checks if time is in interval
      ( ((pt[0]-r) <= bb[0] && (pt[0]+r)>= bb[0]) || ((pt[0]-r) <=bb[1] && (pt[0]+r)>=bb[1]) ||
        ((pt[0]-r) >= bb[0] && (pt[0]+r)<= bb[1]) || ((pt[0]-r) <=bb[0] && (pt[0]+r)>=bb[1]) )&&

      ( ((pt[1]-r) <= bb[2] && (pt[1]+r)>= bb[2]) || ((pt[1]-r) <=bb[3] && (pt[1]+r)>=bb[3]) ||
        ((pt[1]-r) >= bb[2] && (pt[1]+r)<= bb[3]) || ((pt[1]-r) <=bb[2] && (pt[1]+r)>=bb[3]) )&&

      ( ((pt[2]-r) <= bb[4] && (pt[2]+r)>= bb[4]) || ((pt[2]-r) <=bb[5] && (pt[2]+r)>=bb[5]) ||
        ((pt[2]-r) >= bb[4] && (pt[2]+r)<= bb[5]) || ((pt[2]-r) <=bb[4] && (pt[2]+r)>=bb[5]) )   )
	
	result = true;

    return result;
   
}

void 		SKDTREESpliter::build()
{
   SBVHNode root;

   // read file to obtain data
   //read("ABCtest100BASIS");   
   read("ABCtracer");   
/*
   if(toy_skdt == 1)
   {
       Segment s[6];	
       s[0].pt0[0] = 4.00;	  s[3].pt0[0] = 6.00;
       s[0].pt0[1] = 9.00;    s[3].pt0[1] = 4.00;
       s[0].pt0[2] = 0.00;    s[3].pt0[2] = 0.00;
       s[0].pt0[3] = 0.00;    s[3].pt0[3] = 0.00;
       s[0].pt1[0] = 9.00;    s[3].pt1[0] = 9.00;
       s[0].pt1[1] = 10.00;   s[3].pt1[1] = 1.00;
       s[0].pt1[2] = 0.00;    s[3].pt1[2] = 0.00;
       s[0].pt1[3] = 0.00;    s[3].pt1[3] = 0.00;
       s[0].setBbox();	  s[3].setBbox();

       s[1].pt0[0] = 0.00;    s[4].pt0[0] = 1.00;
       s[1].pt0[1] = 7.00;    s[4].pt0[1] = 3.00;
       s[1].pt0[2] = 0.00;    s[4].pt0[2] = 0.00;
       s[1].pt0[3] = 0.00;    s[4].pt0[3] = 0.00;
       s[1].pt1[0] = 7.00;    s[4].pt1[0] = 7.00;
       s[1].pt1[1] = 9.00;    s[4].pt1[1] = 2.00;
       s[1].pt1[2] = 0.00;    s[4].pt1[2] = 0.00;
       s[1].pt1[3] = 0.00;    s[4].pt1[3] = 0.00;
       s[1].setBbox();        s[4].setBbox();

       s[2].pt0[0] = 4.00;    s[5].pt0[0] = 3.00;
       s[2].pt0[1] = 5.00;    s[5].pt0[1] = 2.00;
       s[2].pt0[2] = 0.00;    s[5].pt0[2] = 0.00;
       s[2].pt0[3] = 0.00;    s[5].pt0[3] = 0.00;
       s[2].pt1[0] = 10.00;   s[5].pt1[0] = 6.00;
       s[2].pt1[1] = 7.00;    s[5].pt1[1] = 0.00;
       s[2].pt1[2] = 0.00;    s[5].pt1[2] = 0.00;
       s[2].pt1[3] = 0.00;    s[5].pt1[3] = 0.00;
       s[2].setBbox();        s[5].setBbox();

       for(int i=0; i<6; i++)
       {
           s[i].id = i;
           s[i].tracer_id = i;
           data_idx.push_back(data.size());
	   addSegment(s[i]);
	   updateBbox(bbox, s[i].bbox);
       }
   }
*/
/*
// random segment
   int numb_seg = 1000;
   Segment s[numb_seg];
   for(int idx_rand =0; idx_rand< numb_seg; idx_rand++)
   {

	float h[4];
  	h[0]= (float)rand()/(float)(RAND_MAX/1.0)/1;
	h[1] = (float)rand()/(float)(RAND_MAX/1.0)/1;
	h[2] = (float)rand()/(float)(RAND_MAX/1.0)/1;
 	h[3] = (float)rand()/(float)(RAND_MAX/1.0)/1;

  	for(int j=0; j<4; j++)
	{
	    s[idx_rand].pt0[j] =(float)rand()/(float)(RAND_MAX/1.00); 
	    s[idx_rand].pt1[j] = s[idx_rand].pt0[j] + h[j]; 
	}	
	s[idx_rand].setBbox();
        s[idx_rand].id = idx_rand;
        s[idx_rand].tracer_id = idx_rand;
	data_idx.push_back(data.size());
	addSegment(s[idx_rand]);
	updateBbox(bbox, s[idx_rand].bbox);
       
   }
// end of randon segments

///
///

   //if(debugKDTREE==1)
   //{
   	//cerr << "done reading file" << endl;
        //print(); 
        //cerr << "beginning of issue" <<endl;
   //}

   // Make sure the indeces are the same
   // for debugging purposes
   for(int i =0; i < data.size(); i++)
   {
	   if(data[i].id != data_idx[i] || data[i].id != i)
	   {
		   cerr << "Not matching index at reading" 
		        << " dtat i =" << i 
		   	<< "data id " << data[i].id 
		   	<< "data index id " << data_idx[i]
		   	<< "data tracer id " << data[i].tracer_id<< endl;
	   }
   }

*/
   root.setPrimitivesIndex(getDataIndex(), getDataSize());
   root.setPrimitivesIndex(data, getDataSize());
   //if(debugKDTREE ==1)
   	//cerr << "done seting root primitives indecices" << endl;
   root.setBbox(getDataBbox());
   //root.setSAH();
   root.print();
   root.setParent(-1);
   tree.push_back(root); 

   if(info_skdt == 1)
   {
       initial_num_of_primitives_skdt = getDataSize();
       initial_data_size_skdt = tree[0].getSizeOfData(data);
   }
   tree_size = 1;

   buildTree(tree[0], 0);
   //ofstream ofile("hankSKD");
   //tree[0].Print(ofile, 0, 0, tree);
   tree[0].clear();
   //cerr << "Tree size" << tree.size() << endl;

   //tree[3].print();
   //if(debugKDTREE ==1)
   	//cerr << "done seting building Tree" << endl;
}
int traversalSKD = 0;
int final_traversalSKD = 0;
int num_points_SKDT = 0;

void 		SKDTREESpliter::findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack)
{
	traversalSKD++;
	int test_node = -1;
        if(current_node == 3)
        {
            test_node = tree[current_node].getRightChild();   
	    int n_node = tree[test_node].getNumPrimitives();
	    vector<int> indeces = tree[test_node].getPrimitivesIndex();
            for(int i=0; i<n_node; i++)
 	    {
		//data[indeces[i]].print();
	    }
	}
	int id_left = tree[current_node].getLeftChild();
	int id_right = tree[current_node].getRightChild();
        bool check_left  = false;
        bool check_right = false;
        if(id_left != -2)
	    check_left = isInNode(id_left, pt, r);
	if(id_right != -2)
	    check_right = isInNode(id_right, pt, r);
        //cerr << "The current node is = " << current_node << endl;
	//cerr << "chec_left =" << check_left << endl;
	//cerr << "chec_right =" << check_right << endl;
        //cerr << "id_right = " << id_right << endl;
        if(check_left == true)
        {
	    // check if leaf or number of primitives less than threshold
	    if(tree[id_left].getNumPrimitives() <=  SKDTREE_MIN_NUM_PRIMITIVES || 
              (tree[id_left].getRightChild() == -2 && tree[id_left].getLeftChild() == -2) )
            {
	 	ptr_r++;
		r_stack[ptr_r] = id_left;
            }
	    else
	    {
		current_node = tree[current_node].getLeftChild();
	        findSmallestNodesWithPoint(id_left, pt, r, ptr_r, r_stack);
	    }
            
        }
	if(check_right == true)
        {
	    if(tree[id_right].getNumPrimitives() <=  SKDTREE_MIN_NUM_PRIMITIVES ||
              (tree[id_right].getRightChild() == -2 && tree[id_right].getLeftChild() == -2) )
            {
		//cerr << "should put something in result stack" << endl;
	 	ptr_r++;
		r_stack[ptr_r] = id_right;
            }
	    else
	    {
		current_node = tree[current_node].getRightChild();
	        findSmallestNodesWithPoint(id_right, pt, r, ptr_r, r_stack);
	    }
   	}
	if(check_left == false && check_right == false)
        {
	    ptr_r++;
	    r_stack[ptr_r] = current_node;
	}
}

//*************************************************************************
// This function finds the Node that contain the neighbors of pt
// pt[0] = x, pt[1] = y, pt[2] = z, pt[3] = t
//*************************************************************************
void 		SKDTREESpliter::findNode(float * pt, float r, vector<int> & result)
{
    int * result_stack = new int[10000];
    //int path_stack[10000];
    //int ptr_path = -1;
    int ptr_result = -1;
    //ptr_path++;
    //path_stack[ptr_path] = 0;
    int num_primitives;
    int current_node = 0;
    //int id_left;
    //int id_right;
    //bool check_left;
    //bool check_right;
    //int visit;
    //int traversal = 0;
    clock_t find_while_loop_time, find_for_loop_time;


    find_while_loop_time = clock();
    findSmallestNodesWithPoint(current_node, pt, r, ptr_result, result_stack);
/*
    while(ptr_path > -1)
    {
	traversal++;
	//cerr << "current node = " << current_node << endl;
	//cerr << "In while " << endl;
	id_left = tree[current_node].getLeftChild();
	id_right = tree[current_node].getRightChild();
	//cerr<< "id left =" << id_left << endl;
	//cerr<< "id right =" << id_right << endl;
	if(id_left != -2)
	check_left = isInNode(id_left, pt);
	else
	check_left = false;
	if(id_right != -2)
	check_right = isInNode(id_right, pt);
	else
	check_right = false;
	visit = tree[current_node].getVisit();

        //if(id_left == 1 || id_right == 1)
        //{
	//    cerr << "After group call " << endl;
	//    cerr << "chec_left =" << check_left << endl;
 	//    cerr << "chec_right =" << check_right << endl;
	//    cerr << "visit =" << visit << endl;
	//    cerr << "current node =" << current_node << endl;
	//}

	if(check_left == true && visit == 0)
	{
	    if(tree[id_left].getNumPrimitives() <= 1 || (tree[id_left].getRightChild() == -2 &&
				    			   tree[id_left].getLeftChild() == -2) )
   	    {
	 	ptr_result++;
		result_stack[ptr_result] = id_left;
                tree[id_left].setVisit();
		//cerr << "ptr_result = " << ptr_result << endl;
	    }
	    else
  	    {
		ptr_path++;
		path_stack[ptr_path] = id_left;
		current_node = tree[current_node].getLeftChild();
		//cerr << "ptr_path = " << ptr_path<< endl;
	    }
	}
	else if(check_right == true && visit == 1)
	{
            if(tree[id_right].getNumPrimitives() <= 1 || (tree[id_right].getRightChild() == -2 &&
				    			   tree[id_right].getLeftChild() == -2) )
   	    {
	 	ptr_result++;
		result_stack[ptr_result] = id_right;
                tree[id_right].setVisit();
		//cerr << "ptr_result = " << ptr_result << endl;
	    }
	    else
  	    {
		ptr_path++;
		path_stack[ptr_path] = id_right;
		current_node = tree[current_node].getRightChild();
		//cerr << "ptr_path = " << ptr_path<< endl;
	    }
	}
	else if( visit >= 2)
	{
	    tree[current_node].setVisit();
	    ptr_path--;
	    current_node = path_stack[ptr_path];
	    //cerr << "ptr_path = " << ptr_path<< endl;
	}
    }
*/    
    find_while_loop_time = clock() - find_while_loop_time;

    float radius;
    bool test = false;
    vector<int> indeces;
    
    if(SKDTREEdebuger == 1)
    {
    //int wrong_node = 0;
    cerr << "number of node with result = " << ptr_result+1 << endl;
    cerr << "number of visits in the tree = " << traversalSKD << endl;
    }

    if(traversalSKD >1)
    {
	final_traversalSKD = final_traversalSKD + traversalSKD;
	num_points_SKDT++;
    }
    traversalSKD = 0;

    find_for_loop_time = clock();
    
    for(int stack_idx=0; stack_idx<ptr_result+1; stack_idx++)
    {
	current_node = result_stack[stack_idx];
        // for testing  purposes
	//bool test1 = isInNode(current_node, pt);
        //if(test1 == false)
        //    wrong_node++;	
        // end of testing 
	//current_node = tree[current_node].getParent();
        num_primitives = tree[current_node].getNumPrimitives();

	clock_t node_time, indeces_time;
        if(SKDTREEdebuger == 1)
 	{
	//cerr << "after while " << endl;
	//cerr << "num primitives =" << num_primitives << endl;
	//tree[current_node].print();
        //indeces_time = clock();
	}
	indeces = tree[current_node].getPrimitivesIndex();
        
        if(SKDTREEdebuger == 1)
	{
	//indeces_time = clock() - indeces_time
	//cerr << time to get indeces of node << current << " time = " << indeces_time << endl;
        //node_time = clock();
	}
	// check that the resul is in the limits
	// if not it goes pack to the parent 
	for(int i =0; i<num_primitives; i++)
	{
            int id_result = indeces[i];
     	    test = data[id_result].findPosition(pt);      	
	    // debugging
	    //if(data[indeces[i]].id == 3339)
	    //{
	    //	cerr << "The result is here" << endl;      	
	    //	cerr << "The test is = " << test << endl;
            //    data[indeces[i]].print();
	    //	cerr << "point in find Node = ( " << pt[0] << ", "<< pt[1] << ", " << pt[2] << ", " << pt[3] << ")" << endl;
            //    radius = data[indeces[i]].getRadius(pt);
            //    cerr << "The radius is = " << radius << endl;
	    //}
            // end debuging
	    //cerr << "test =" << test << endl;
	    if(test == true)
	    {
                radius = data[id_result].getRadius(pt);
		if(radius <= r)
 	           result.push_back(id_result);
	    }
	}
        if(SKDTREEdebuger == 1)
	{

        }
    }

    // check for radius and grow
    //cerr << "Wrong node  = " << wrong_node<<endl;
    if(SKDTREEdebuger == 1)
    {

    find_for_loop_time = clock() - find_for_loop_time;
    cerr << "find_while_loop_time = " << find_while_loop_time << endl;
    cerr << "find_for_loop_time = " << find_for_loop_time<< endl;
    }
    // free result and path stack
    //delete [] result_stack;
    //delete [] path_stack;
}

//*************************************************************
// This is the traditional find slow for loop 
// fro comparaison purposes
//**************************************************************
void 		SKDTREESpliter::traditionalFind(float * pt, float r, vector<int> & result)
{
    //Particle main_p(pt[0], pt[1], pt[2], pt[3]); 
    for(int i=0; i < getDataSize(); i++)
    {
	bool test = data[i].findPosition(pt);      	
	//cerr << "test =" << test << endl;
	if(test == true)
	{
	    float radius = data[i].getRadius(pt);
	    if(radius <= r)
	    {
		result.push_back(i);
		//cerr << "traditional radius = " << radius << endl;
	        //cerr << "radius to compare to is = " << r << endl;
		//cerr << "point in traditional find is = ( " << pt[0] << ", "<< pt[1] << ", " << pt[2] << ", " << pt[3] << ")" << endl;
		//data[i].print();
	    }
	}
    }
}



//*************************************************************
// testing while I go
//*************************************************************
void 		SKDTREESpliter::test()
{
    float build_time_sec, search_time_sec;
    clock_t build_time, search_time;
    static float pt[4] = {1.00, 1.00, 1.00, 4.8};
    static float temp_pt[4] = {2.00, 3.00, 0.00, 0.0};
    vector<int> result_0;
    vector<int> result_1;

    build_time = clock();
    build();
    build_time = clock() - build_time;
    build_time_sec = ((float)build_time)/CLOCKS_PER_SEC;
 
// create point that are unformly distributed in space
    int numb = 10;
    float X[numb];
    float Y[numb];
    float Z[numb];
    float T[numb];
    float * bb_node0 = tree[0].getBbox();


    float deltaX = (bb_node0[1]- bb_node0[0])/numb;
    float deltaY = (bb_node0[3]- bb_node0[2])/numb;
    float deltaZ = (bb_node0[5]- bb_node0[4])/numb;
    float deltaT = (bb_node0[7]- bb_node0[6])/numb;

    X[0] = 0.0;
    Y[0] = 0.0;
    Z[0] = 0.0;
    T[0] = 0.0;
    for(int i=1; i<numb; i++)
    {
	X[i] = X[i-1]+ deltaX;
	Y[i] = Y[i-1]+ deltaY;
	Z[i] = Z[i-1]+ deltaZ;
	T[i] = T[i-1]+ deltaT;
    }

    float point[numb*numb*numb*numb][4];
    int idx = 0;
    for(int i=1; i<numb; i++)
    {
	for(int j=0; j<numb; j++)
	{
	    for(int k=0; k<numb; k++)
	    {
		for(int m=0; m<numb; m++)
		{
		    point[idx][0] = X[i]; 
		    point[idx][1] = Y[j]; 
		    point[idx][2] = Z[k]; 
		    point[idx][3] = T[m]; 
		    idx++;
		}
	    }
	}
    }
// end of creation of unformely distributed point

    search_time = clock();

    for(int i=0; i< numb*numb*numb*numb; i++)
        findNode(point[i], .050, result_0);
        //findNode(temp_pt, 1.00, result_0);

    search_time = clock() - search_time;
    search_time_sec = ( (float)search_time )/CLOCKS_PER_SEC;

    for(int i=0; i< numb*numb*numb*numb; i++)
        traditionalFind(point[i], .050, result_1);
        //traditionalFind(temp_pt, 1.00, result_1);
    cout << "*********************SKDTREE Results*********************" << endl;
    cout << "Build time = " << build_time_sec << "		Search time =" << search_time_sec << endl;
    cout << "size of result 0 = " << result_0.size() <<endl;
    cout << "size of result 1 = " << result_1.size() <<endl;
    cout << "tree size = " << tree.size() << endl;

    int size_of_tree = tree[0].getSizeOfTree(tree);
    int size_of_data = tree[0].getSizeOfData(data); 
    int num_of_primitives = data.size();
    int num_of_nodes = tree.size();
    cout << "size of data in byte = " << size_of_data<< endl;
    cout << "size of tree in byte = " <<size_of_tree<< endl;


    // save in files
    fstream file;
    file.open("SKDTfile", fstream::in | fstream::out | fstream::app);
    file <<  initial_num_of_primitives_skdt  << "\t";
    file <<  initial_data_size_skdt << "\t";
    file <<  num_of_primitives << "\t";
    file <<  size_of_data << "\t";
    file <<  num_of_nodes << "\t";
    file <<  size_of_tree << "\t";
    file <<  build_time_sec << "\t";
    file <<  search_time_sec << "\t";
    file <<  ((float)final_traversalSKD)/((float)num_points_SKDT) << "\n";
    file.close();

/*
// debugging
    int test_id = 3339; 
    int n_prev = tree[0].getNumPrimitives();
    int result = -1;
    for(int i=1; i< tree.size(); i++)
    {
        int n = tree[i].getNumPrimitives();
        vector<int> idx_test = tree[i].getPrimitivesIndex();
	for(int j=0; j< n; j++)
	{
 	    if(data[idx_test[j]].id == test_id && n <= n_prev) 
	    {
		result = i;
		n_prev = n;		
	    }
	}
    }
    cerr << "--------------- segment ------------" << endl;
    data[test_id].print();
    cerr << "is locate in node  = "<< result << endl;
    int n_node = tree[1].getNumPrimitives();
    for(int i=0; i< numb*numb*numb*numb; i++)
    {
        bool is_in_node = isInNode(1, point[i]);
        if(is_in_node == true)
        {
	    vector<int> ids = tree[1].getPrimitivesIndex();
            for(int j=0; j< n_node; j++)
	    {
                bool is_good_seg = data[ids[j]].findPosition(point[i]);
 		if(is_good_seg == true)
 		{
		    float radius = data[ids[j]].getRadius(point[i]);
		    if(radius <= .50)
		    {
			data[ids[j]].print();
	                cerr << "What is going on " << endl;
			cerr << "point ( "<< point[i][0]<< ", " << point[i][1] << ", " << point[i][2] << ", " << point[i][3] << ")" << endl;
		    }
		}
	    }
	}
    }

*/
/*
for(int i =0; i<num_primitives; i++)
	{
	    //cerr << "test =" << test << endl;
	    if(test == true)
	    {
                radius = data[indeces[i]].getRadius(pt);
		if(radius <= r)
 	           result.push_back(indeces[i]);
	    }
	}
*/
// end of debugging    
    int tree_size = 0;
    //getTreeSizeInByte(tree[0], tree_size);
    //int size_of_data = sizeof(data) + getSizeInByte();
    //int size_of_tree = sizeof(tree) + tree_size; 
    //cout << "size of data in byte = " << size_of_data<< endl;
    //cout << "size of tree in byte = " <<size_of_tree<< endl;

   
    for(int i=0; i < result_0.size(); i++)
    	cout << result_0[i] << ", ";
    cout<< endl;
    for(int i=0; i < result_1.size(); i++)
    	cout << result_1[i] << ", ";
    cout<< endl;

   }
