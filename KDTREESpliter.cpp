#include <KDTREESpliter.h>
#include <SBVHNode.h>
#include <primitive.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

// for debugging purposes
float KDTdebuger = 0;

using namespace std;

KDTREESpliter::KDTREESpliter()
{
    tree_size = -10;
    tree.resize(0);
}
KDTREESpliter::~KDTREESpliter()
{}

// *****************************************
// This function calculates SAH
//******************************************
float KDTREESpliter::calculateSAH(float * bbox)
{
    float sah = abs( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[1] - bbox[0]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[3] - bbox[2]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[5] - bbox[4]) ) ;
    return sah;
}


//*****************************************************************
// bb1 --> bbox1
// bb2 --> bbox2
// This function takes 2 bbox and updates the first to a new bbox
//*****************************************************************
void KDTREESpliter::updateBbox(float * bb1, float * bb2)
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
void KDTREESpliter::append(vector<int>& arr, int id)
{
    arr.push_back(id);
}

void KDTREESpliter::append(vector<Segment>& arr, Segment & id)
{
    arr.push_back(id);
}

/*
//*********************************************************************
// This updates all the ancesters after a split is performed
// to make sure node conserves there initegraty
//**********************************************************************
void KDTREESpliter::updateAncesters(SBVHNode & node, Segment s, int idx)
{
    int parent_idx = node.getParent();
    node.append(idx);
    node.append(s);
    cerr << "idx =" << idx << endl; 
    while(parent_idx > -1)
    {
	//tree[parent_idx].append(s);
	tree[parent_idx].append(idx);
	parent_idx = tree[parent_idx].getParent();
    }
}
*/
bool		KDTREESpliter::lessThanMinimumVolume(float * bb)
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
bool		KDTREESpliter::isOverlaped(float * bb, int dim, float s)
{
    bool result = false;

    if((bb[2*dim+1] - bb[2*dim+0]) >0 && bb[2*dim+0] < s && s<bb[2*dim+1])
	result = true;
    //else if((bb[2*dim+1] - bb[2*dim+0]) <0  && bb[2*dim+1] < s && s < bb[2*dim+1])
	//result = true;
    return result;
}
bool		KDTREESpliter::isOverlaped(float *  bb, int dim, float * s)
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
bool		KDTREESpliter::isOverlaped(float *  bb, float * s)
{
    bool result = false;
    if( (bb[1] - bb[0]) >0 && (bb[0] <= s[0] && s[0] <= bb[1]) &&
        (bb[3] - bb[2]) >0 && (bb[2] <= s[2] && s[2] <= bb[3]) &&
        (bb[5] - bb[4]) >0 && (bb[4] <= s[4] && s[4] <= bb[5]) &&
        (bb[7] - bb[6]) >0 && (bb[6] <= s[6] && s[6] <= bb[7]) )
	result = true;
    else if( (bb[1] - bb[0]) >0 && (bb[0] <= s[1] && s[1] <= bb[1]) &&
             (bb[3] - bb[2]) >0 && (bb[2] <= s[3] && s[3] <= bb[3]) &&
             (bb[5] - bb[4]) >0 && (bb[4] <= s[5] && s[5] <= bb[5]) &&
             (bb[7] - bb[6]) >0 && (bb[6] <= s[7] && s[7] <= bb[7]) )
	result = true;
    else if( (s[1] - s[0]) >0 && (s[0] <= bb[1] && bb[1] <= s[1]) &&
             (s[3] - s[2]) >0 && (s[2] <= bb[3] && bb[3] <= s[3]) &&
             (s[5] - s[4]) >0 && (s[4] <= bb[5] && bb[5] <= s[5]) &&
             (s[7] - s[6]) >0 && (s[6] <= bb[7] && bb[7] <= s[7]) )
	result = true;
    else if( (s[1] - s[0]) >0 && (s[0] <= bb[1] && bb[1] <= s[1]) &&
             (s[3] - s[2]) >0 && (s[2] <= bb[3] && bb[3] <= s[3]) &&
             (s[5] - s[4]) >0 && (s[4] <= bb[5] && bb[5] <= s[5]) &&
             (s[7] - s[6]) >0 && (s[6] <= bb[7] && bb[7] <= s[7]) )
	result = true;    
    return result;
}
/*
//*************************************************************
// Uses preordertraversal to claculate the size of tree
//*************************************************************
void 		KDTREESpliter::getTreeSizeInByte(SBVHNode & root, int & result)
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
void 	KDTREESpliter::findBestSpatialSplit(ObjectSplitTree & result, SBVHNode & node)
{
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
    result.Cost = 1000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;
    result.split_location = -10;
    //bool right_bound = true;

    // split_case = 0  --> we consider segements lower bound for the splits
    // split case = 1  --> we consider the segments upper bound for splits 
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
    
    // determine step size to consider
    if(n_node > 1000)
    	step = floor(n_node/1000);
	//step =1;

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
	
	    if(KDTdebuger == 1)
            {
	    //cerr << "idx_node =" << idx_node << endl; 
	    //node.node_spec[idx_node].print();
            }
       	    seg_bb = node.node_spec[idx_node].getBbox();

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
            // set up range to determinw overlaps
	    if((n_node - idx_node) > 1000)
	    	threshold_num = idx_node + 1000;
	    else
	    	threshold_num = n_node;

	    if(idx_node >1000)
		threshold_num_begin =idx_node - 1000;
	    else  
		threshold_num_begin =0;
	    //temp_Pl= 0;
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
	    temp_Pl =0;
            P_span = 0;
            for(int idx_tracer= threshold_num_begin; idx_tracer < threshold_num; idx_tracer++)
	    {
		//counting++;
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 
		float center = node.node_spec[idx_tracer].getCenter(idx_dim);

		if(temp_bb[idx_dim*2+0] < temp_split_location && temp_split_location < temp_bb[idx_dim*2+1])
                {
		     //P_span++;
		     if(center < temp_split_location)
			temp_Pl++;
		     else
			temp_Pr++;
		}
	        //if(temp_split_location>=temp_bb[idx_dim*2+1])
	        //     Pl++;
	    }

            // calculation of number of nodes on each side
            //Pl = idx_node + P_span;
            Pl = idx_node + temp_Pl;
	    //Pr = n_node - idx_node + P_span;
	    Pr = n_node - idx_node + temp_Pr;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    C = ((Pl)*left_sah + (Pr)*right_sah) / sah;

            left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
            right_range = abs(temp_split_location - bbox[idx_dim*2+1]);
            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            ///cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
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
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
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
        //cerr << "sort dim = " << result.sort_dim << endl;
        //cerr << "node bbox =" << bbox[2*result.sort_dim] << "__" << bbox[2*result.sort_dim+1] << endl; 

	label : ;
        }// end of for loop
      }// end of while
      //if(result.split_location == bbox[2*result.sort_dim + 0] || result.split_location == bbox[2*result.sort_dim + 1])
      if(abs(result.split_location- bbox[2*result.sort_dim + 0]) < 0.00001 || abs(result.split_location - bbox[2*result.sort_dim + 1]) < .00001)
      {
        cerr << " Not suppose to occur idx dim = " << idx_dim << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(0);
      }
    }
    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0])
    {
	 cerr << "split location" << result.split_location << "is outside of the bounds"<< endl;
 	 cerr << "BNot suppose to occur " << endl;
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 exit(0);
    }
    // end of debugging stuff
    if(result.sort_dim < 0 || result.sort_dim>3)
    {
	cerr << "not suppose to occur" << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(0);
    }

    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);

/*
    //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();

    float range = 0;
    result.Cost = n_node * 100000;
    result.small_to_split = true;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	// initialize number of primitives on each side
	Pl = 0;
	Pr = 0;

	// sort By axis
    	if(idx_dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    	if(idx_dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    	if(idx_dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    	if(idx_dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

	//cerr << "After sort " << endl;
	//cerr << "range = " << abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) << endl;
	range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
if(range > .001)
  {
        // stets step size for looping thorugh the bbox
	float delta = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2])/ 100;
	//for(float idx_bbox = bbox[idx_dim*2+0]+delta; idx_bbox <bbox[idx_dim*2+1]; idx_bbox = idx_bbox+delta)
	for(float idx_bbox = bbox[idx_dim*2+0]+delta; idx_bbox <(bbox[idx_dim*2+0]+ 1*delta); idx_bbox = idx_bbox+delta)
	{
	
	    //iset the temp bbox
	    float left_bbox[8];
	    float  right_bbox[8];
	    for(int i = 0 ; i < 4; i++)
	    {
		if(idx_dim == i)
		{
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = idx_bbox;
		    right_bbox[i*2+0] = idx_bbox;
		    right_bbox[i*2+1] = bbox[i*2+1];
		    //cerr << left_bbox[2*i+0] << "ssss" << left_bbox[i*2+1] << endl;
		}
		else
		{
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
	    }
	    // count primitives in each bbox
	    for(int idx_tracer=0; idx_tracer < n_node; idx_tracer++)
            {
		//node.node_spec[idx_tracer].print();
        	int id = node.node_spec[idx_tracer].id;
		bool left_overlap = isOverlaped(left_bbox, data[id].getBbox());
		bool right_overlap = isOverlaped(right_bbox, data[id].getBbox());
		if(left_overlap == true)
		    Pl++;
		if(right_overlap == true)
		    Pr++;
	    }

	    //cerr << "Pl = " << Pl << endl;
	    int ntest = Pl + Pr;
	    if((Pl == 0 || Pr == 0) && ntest < n_node )
	    {
	        cerr << "Pr = " << Pr << endl;
	        cerr << "Pl = " << Pl << endl;
	    	cerr << "number of primitives in node =" << n_node << endl;
	    }

	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
            float sah = node.getSAH();

	    // calculte cost update 
            C = (Pl*left_sah + Pr*right_sah) / sah;

	    //cerr << "Cost =" << C << endl;
	    //cerr << "left_sah =" << left_sah << endl;
	    //cerr << "right_sah =" << right_sah << endl;
	    //cerr << "sah =" << sah << endl;

	    // update cost
	    if(C < result.Cost)
	    {
		//cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
	        result.Cost = C;
	        result.sort_dim = idx_dim;
	        result.split_location = idx_bbox;
		result.small_to_split = false;
	    }
	    Pl = 0;
	    Pr = 0;
	}
  } // end of if
    }
    if(result.small_to_split == true)
    {
	int dim = -1;
        for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    	{
	    if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	    {
            	dim  = idx_dim;
	    	range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	    }
           // maybe i need to normalize time
         }
         float center = (bbox[dim*2+1]+bbox[dim*2+0])/2;
	 result.Cost = 0;
	 result.sort_dim = dim;
	 result.split_location = center;
    }

    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0])
    {
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 cerr << "split locatioin" << result.split_location << endl;
    }
    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
    //cerr << "Split location =" <<result.split_location << endl;
    //cerr << "Split dim = " << result.sort_dim << endl;;
    //result.print();
*/  
}

//**************************************************************
// This use sah to find the best Split 
// for the data
//**************************************************************
void 	KDTREESpliter::findBestObjectSplit(ObjectSplitTree & result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();

    float range = 0;
    result.Cost = n_node * 100000;
    result.small_to_split = true;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	// initialize number of primitives on each side
	Pl = 0;
	Pr = 0;

	// sort By axis
    	if(idx_dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    	if(idx_dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    	if(idx_dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    	if(idx_dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

	//cerr << "After sort " << endl;
	//cerr << "range = " << abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) << endl;
	range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
/*
	for(int idx_tracer=0; idx_tracer < n; idx_tracer++)
	{
	    
            int id = node.node_spec[idx_tracer].id;
	    bool overlap = isOverlaped(bbox, idx_dim, data[id].getBbox());
	    if(overlap == false)
	    {
		//cerr << "dim = " << idx_dim << endl;
		//cerr << "original = " << bbox[2*idx_dim] << "____" << bbox[2*idx_dim+1] << endl;
		//node.print();
		//data[id].print();
	    }
	}
*/

if(range > .001)
  {
        // stets step size for looping thorugh the bbox
	float delta = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2])/ 100;
	//for(float idx_bbox = bbox[idx_dim*2+0]+delta; idx_bbox <bbox[idx_dim*2+1]; idx_bbox = idx_bbox+delta)
	for(float idx_bbox = bbox[idx_dim*2+0]+delta; idx_bbox <(bbox[idx_dim*2+0]+ 1*delta); idx_bbox = idx_bbox+delta)
	{
	
	    //iset the temp bbox
	    float left_bbox[8];
	    float  right_bbox[8];
	    for(int i = 0 ; i < 4; i++)
	    {
		if(idx_dim == i)
		{
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = idx_bbox;
		    right_bbox[i*2+0] = idx_bbox;
		    right_bbox[i*2+1] = bbox[i*2+1];
		    //cerr << left_bbox[2*i+0] << "ssss" << left_bbox[i*2+1] << endl;
		}
		else
		{
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
	    }
	    // count primitives in each bbox
	    for(int idx_tracer=0; idx_tracer < n_node; idx_tracer++)
            {
		//node.node_spec[idx_tracer].print();
        	int id = node.node_spec[idx_tracer].id;
		//bool left_overlap = isOverlaped(left_bbox, idx_dim, data[id].getCenter(idx_dim));
		//bool right_overlap = isOverlaped(right_bbox, idx_dim, data[id].getCenter(idx_dim));

		//if(left_overlap == true)
		if(data[id].getCenter(idx_dim) <= idx_bbox)
		    Pl++;
		//if(right_overlap == true)
		else
		    Pr++;
/*
		if(left_overlap == false && right_overlap == false)
		{
		cerr << "dim = " << idx_dim << endl;
		cerr << "original = " << bbox[2*idx_dim] << "____" << bbox[2*idx_dim+1] << endl;
		cerr << "left  = " << left_bbox[2*idx_dim] << "____" << left_bbox[2*idx_dim+1] << endl;
		cerr << "right = " << right_bbox[2*idx_dim] << "____" << right_bbox[2*idx_dim+1] << endl;
		cerr << "left_overlap"<<left_overlap << endl;
		cerr << "right_overlap"<<right_overlap << endl;
		cerr << "Pl =" << Pl << endl;
		cerr << "Pr =" << Pr << endl;
		//data[id].print();
		}
*/
	    }

	    //cerr << "Pl = " << Pl << endl;
	    int ntest = Pl + Pr;
	    if((Pl == 0 || Pr == 0) && ntest < n_node )
	    {
	        cerr << "Pr = " << Pr << endl;
	        cerr << "Pl = " << Pl << endl;
	    	cerr << "number of primitives in node =" << n_node << endl;
	    }

	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
            float sah = node.getSAH();

	    // calculte cost update 
            C = (Pl*left_sah + Pr*right_sah) / sah;

	    //cerr << "Cost =" << C << endl;
	    //cerr << "left_sah =" << left_sah << endl;
	    //cerr << "right_sah =" << right_sah << endl;
	    //cerr << "sah =" << sah << endl;

	    // update cost
	    if(C < result.Cost)
	    {
		//cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
	        result.Cost = C;
	        result.sort_dim = idx_dim;
	        result.split_location = idx_bbox;
		result.small_to_split = false;
	    }
	    Pl = 0;
	    Pr = 0;
	}
  } // end of if
    }
    if(result.small_to_split == true)
    {
	int dim = -1;
        for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    	{
	    if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	    {
            	dim  = idx_dim;
	    	range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	    }
           // maybe i need to normalize time
         }
         float center = (bbox[dim*2+1]+bbox[dim*2+0])/2;
	 result.Cost = 0;
	 result.sort_dim = dim;
	 result.split_location = center;
    }

    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0])
    {
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 cerr << "split locatioin" << result.split_location << endl;
    }
    
    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
    //cerr << "Split location =" <<result.split_location << endl;
    //cerr << "Split dim = " << result.sort_dim << endl;;
    //result.print();
}


//********************************************************
// 
//********************************************************
void KDTREESpliter::findObjectSplit1(ObjectSplitTree &result, SBVHNode & node)
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
    
    // sort By axis
    if(dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    if(dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    if(dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    if(dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());


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

void KDTREESpliter::findObjectSplitMedian(ObjectSplitTree & result, SBVHNode & node)
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
void KDTREESpliter::performSplit(ObjectSplitTree & split, SBVHNode & node, SBVHNode & NL, SBVHNode & NR)
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
    float * l_bbox = new float[8];
    float * r_bbox = new float[8];

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
    //cerr << "split pos =" << split_pos << endl;

    //if(debugKDTREE ==1)
    	//cerr << "I have pasted the initialization" << endl; 
    int id;
    vector<double> line_center;
    float * bb;
    bool overlap_left;
    bool overlap_right;
    float center;

    for(int i =0; i<n; i++)
    {
        id = node.node_spec[i].id;
        //node.node_spec[i].print();
 	center = node.node_spec[i].getCenter(dim);
        bb = node.node_spec[i].getBbox();
        overlap_left = isOverlaped(l_bbox, bb);
        overlap_right = isOverlaped(r_bbox, bb);
	bool overlap_main = isOverlaped(bbox, bb);

	if(overlap_main == false)
	{
        node.node_spec[i].print();
        for(int m=0; m<4; m++)
	{
	     cerr << l_bbox[m*2+0] << "__" << l_bbox[m*2+1] << ",  " ;
	}
	cerr << "" << endl;
        for(int m=0; m<4; m++)
	{
	     cerr << r_bbox[m*2+0] << "__" << r_bbox[m*2+1] << ",  " ;
	}
	cerr << "" << endl;

	cerr << "at least 1 of them should be  true" << endl;
	cerr << "overlap  left =" << overlap_left << endl;
	cerr << "overlap  right =" << overlap_right<< endl;
	cerr << "overlap  main =" << overlap_main<< endl;
        }
        if(overlap_left == true)
    	{
  	   append(l_index, id);
  	   append(l_node_spec, data[id]);
           //updateBbox(l_bbox, bb2);	
	}
	if(overlap_right == true)
	{
  	   append(r_index, id);
	   append(r_node_spec, data[id]);
           //updateBbox(r_bbox, bb2);
	}
    } 

    //if(debugKDTREE == 1)
        //cerr << "After updating bounding box and ..." << endl; 

    NL.setBbox(l_bbox);
    //NL.setSAH();
    NL.setPrimitivesIndex(l_index, l_index.size());
    NL.setPrimitivesIndex(l_node_spec, l_index.size());
    //NL.setNumPrimitives(l_index.size());

    NR.setBbox(r_bbox);
    //NR.setSAH();
    NR.setPrimitivesIndex(r_index, r_index.size());
    NR.setPrimitivesIndex(r_node_spec, r_index.size());
    //NR.setNumPrimitives(r_index.size());
    
    //if(debugKDTREE == 1)
    {
    	//cerr << "Left child Primitives =" << NL.getNumPrimitives() << endl;
    	//cerr << "Right Child Primitives="<<  NR.getNumPrimitives() << endl;
    	//cerr << "End of perform Split ..." << endl; 
    }
    //cerr << "Leaving perform split" << endl;

}

//******************************************************************************************
// 
//******************************************************************************************
void KDTREESpliter::buildTree(SBVHNode & node, int me)
{

     int  n = node.getNumPrimitives();
/*
     // testing
     float * t_bb = node.getBbox();
     for(int i=0; i<n; i++)
     {
      float * bb = node.node_spec[i].getBbox();
      bool test = isOverlaped(t_bb, bb);
     if(test == false)
     {
        node.node_spec[i].print();
	node.print();
     }
     }
     // end testing for debuggin
*/
     //cerr << "In the buildTree" << endl;
     ObjectSplitTree split;

     SBVHNode nodeL;
     SBVHNode nodeR;


     nodeL.setParent(me);
     nodeR.setParent(me);
     node.setLeftChild(tree.size());
     int l = node.getLeftChild();
     node.setRightChild(tree.size()+1);
     int r = node.getRightChild();

     cerr << "l =" << l << " r =" << r << endl;
     //cerr << "me =" << node.getMe() << endl;

     //cerr << "In  the middle" << endl;

    // this just allocate memory for the cheildren    
    //node->allocateChildren();
    //findBestSplit(split, node);
    //findBestObjectSplit(split, node);
    //findBestSpatialSplit(split, node);
    //findObjectSplit1(split, node);
    findObjectSplitMedian(split, node);
    performSplit(split, node, nodeL, nodeR);

    tree.push_back(nodeL);
    tree.push_back(nodeR);
    int size = tree.size();

    //cerr << "Left child Primitives =" << tree[l].getNumPrimitives() << endl;
    //cerr << "Right Child Primitives="<< tree[r].getNumPrimitives() << endl;

    if(tree[l].getNumPrimitives() > 20)
    {
        //cerr << "conditional 1" << endl;
    	buildTree(tree[l], l);
    }
    else
    {
        tree[l].setLeftChild(-2);
        tree[l].setRightChild(-2);
    }
    if(tree[r].getNumPrimitives() > 20)
    {
        //cerr << "conditional 2" << endl;
    	buildTree(tree[r], r);
    }
    else
    {
        tree[r].setLeftChild(-2);
        tree[r].setRightChild(-2);
    }

/*
    if(l==3 || r == 3)
    {
        tree[l].print();
	tree[r].print();
    }
*/
    //cerr << "leaving Build tree" << endl;
    // clean the shallow copy
    // clean nodes too
    tree[l].clear();
    tree[r].clear();
}

//***********************************************************
//checks if the Node is in the Tree
//************************************************************
bool		KDTREESpliter::isInNode(int node_idx, float * pt)
{
    bool result = true;
    //cerr << "checking if it is in the node" << endl;
    //cerr << "number of tracers =" << tree[node_idx].getNumPrimitives() << endl;
    //cerr << "node locatioin =" << node_idx << endl;
    float *  bb = tree[node_idx].getBbox();
    float delta0 = .001;
    float delta1 = .002;
    //tree[node_idx].print();
    if( (abs(bb[1] -bb[0])> delta1 && (bb[0] <= pt[0] && pt[0] <= bb[1])) && 
	(abs(bb[3] -bb[2])> delta1 && (bb[2] <= pt[1] && pt[1] <= bb[3])) &&
        (abs(bb[5] -bb[4])> delta1 && (bb[4] <= pt[2] && pt[2] <= bb[5])) &&
	(abs(bb[7] -bb[6])> delta1 && (bb[6] <= pt[3] && pt[3] <= bb[7])) )
	result = true;

    else if( (abs(bb[1] -bb[0])> delta1 && (bb[0] <= pt[0]+delta0 && pt[0]+delta0 <= bb[1])) && 
	     (abs(bb[3] -bb[2])> delta1 && (bb[2] <= pt[1]+delta0 && pt[1]+delta0 <= bb[3])) &&
             (abs(bb[5] -bb[4])> delta1 && (bb[4] <= pt[2]+delta0 && pt[2]+delta0 <= bb[5])) &&
	     (abs(bb[7] -bb[6])> delta1 && (bb[6] <= pt[3]+delta0 && pt[3]+delta0 <= bb[7])) )
        result = true; 

    else if( (abs(bb[1] -bb[0])> delta1 && (bb[0] <= pt[0]-delta0 && pt[0]-delta0 <= bb[1])) && 
	     (abs(bb[3] -bb[2])> delta1 && (bb[2] <= pt[1]-delta0 && pt[1]-delta0 <= bb[3])) &&
             (abs(bb[5] -bb[4])> delta1 && (bb[4] <= pt[2]-delta0 && pt[2]-delta0 <= bb[5])) &&
	     (abs(bb[7] -bb[6])> delta1 && (bb[6] <= pt[3]-delta0 && pt[3]-delta0 <= bb[7])) )
	result = true;
/*
    if( abs(bb[1]-bb[0])> delta1 && ( ((pt[0]+delta0)<bb[1] && (pt[0]+delta0)>bb[0]) ||  ((pt[0]-delta0)<bb[1] && (pt[0]-delta0)>bb[0]) ) )
    	result = true;
    else if( abs(bb[3]-bb[2])> delta1 && ( ((pt[1]+delta0)<bb[3] && (pt[1]+delta0)>bb[2]) ||  ((pt[1]-delta0)<bb[3] && (pt[1]-delta0)>bb[2]) ) )
    	result = true;
    else if( abs(bb[5]-bb[4])> delta1 && ( ((pt[2]+delta0)<bb[5] && (pt[2]+delta0)>bb[4]) ||  ((pt[2]-delta0)<bb[5] && (pt[2]-delta0)>bb[4]) ) )
    	result = true;
    else if( abs(bb[7]-bb[6])> delta1 && ( ((pt[3]+delta0)<bb[7] && (pt[3]+delta0)>bb[6]) ||  ((pt[3]-delta0)<bb[7] && (pt[3]-delta0)>bb[6]) ) )
    	result = true;
    else if( abs(bb[1]-bb[0])> delta1 && ( ((pt[0])<bb[1] && (pt[0])>bb[0]) || ((pt[0])<bb[1] && (pt[0])>bb[0]) ) )
    	result = true;
    else if( abs(bb[3]-bb[2])> delta1 && ( ((pt[1])<bb[3] && (pt[1])>bb[2]) ||  ((pt[1])<bb[3] && (pt[1])>bb[2]) ) )
    	result = true;
    else if( abs(bb[5]-bb[4])> delta1 && ( ((pt[2])<bb[5] && (pt[2])>bb[4]) ||  ((pt[2])<bb[5] && (pt[2])>bb[4]) ) )
    	result = true;
    else if( abs(bb[7]-bb[6])> delta1 && ( ((pt[3])<bb[7] && (pt[3])>bb[6]) ||  ((pt[3])<bb[7] && (pt[3])>bb[6]) ) )
    	result = true;
    
*/
    //cerr << "is in Node "<< node_idx << " result =" << result << endl;
    //cerr << "done checking" << endl;
    return result;
}

void 		KDTREESpliter::build()
{
   SBVHNode root;

   // read file to obtain data
   //read("ABCtest100BASIS");   
   read("ABCtracer");   

   //if(debugKDTREE==1)
   {
   	//cerr << "done reading file" << endl;
        //print(); 
        //cerr << "beginning of issue" <<endl;
   }

   root.setPrimitivesIndex(getDataIndex(), getDataSize());
   root.setPrimitivesIndex(data, getDataSize());
   //if(debugKDTREE ==1)
   	//cerr << "done seting root primitives indecices" << endl;
   root.setBbox(getDataBbox());
   //root.setSAH();
   //if(debugKDTREE ==1)
   	//cerr << "done seting root parent to NULL" << endl;
   
   root.setParent(-1);
   tree.push_back(root);
   tree_size = 1;
   //cerr << "l child =" << tree[0].getLeftChild() << endl; 
   //tree[0].print();
   //append(root);

   buildTree(tree[0], 0);
   tree[0].clear();
   //tree[3].print();
   //if(debugKDTREE ==1)
   	//cerr << "done seting building Tree" << endl;
}

//*************************************************************************
// This function finds the Node that contain the neighbors of pt
// pt[0] = x, pt[1] = y, pt[2] = z, pt[3] = t
//*************************************************************************
void 		KDTREESpliter::findNode(float * pt, float r, vector<int> & result)
{
    int h = 100;
    int result_stack[h*100];
    int path_stack[h];
    int ptr_path = -1;
    int ptr_result = -1;
    int n = tree[0].getNumPrimitives();
    ptr_path++;
    path_stack[ptr_path] = 0;

    // root 
    //int dim = tree[0].getSplitDim();
    int num_primitives; //= tree[0].getNumPrimitives();
    int current_node = 0;
    //float split_location = tree[0].getSplitLocation();
    int id_left;
    int id_right;
    bool check_left;
    bool check_right;
    int visit;
    //cerr << "Split location = " << split_location << endl; 

    clock_t find_while_loop_time, find_for_loop_time;
    
    find_while_loop_time = clock() ;
    //is the point in the node
    //while(isInNode(node_id, pt) == true && num_primitives > 3 )
    while(ptr_path > -1)
    {
        cerr << "Debuging Line -----" << endl;
	id_left = tree[current_node].getLeftChild();
	id_right = tree[current_node].getRightChild();
	if(id_left != -2)
	    check_left = isInNode(id_left, pt);
	else
	    check_left = false;
	if(id_right != -2)
	    check_right = isInNode(id_right, pt);
	else
	    check_right = false;
	visit = tree[current_node].getVisit();
	//cerr << "After group call " << endl;
	//cerr << "chec_left =" << check_left << endl;
	//cerr << "chec_right =" << check_right << endl;
  	//cerr << "visit =" << visit << endl;
	//cerr << "current node =" << current_node << endl;
	if(check_left == true && visit == 0)
	{
	    if(tree[id_left].getNumPrimitives() <= 20 || (tree[id_left].getRightChild() == -2 &&
				    			  tree[id_left].getLeftChild() == -2) )
   	    {
	 	ptr_result++;
		result_stack[ptr_result] = id_left;
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
            if(tree[id_right].getNumPrimitives() <= 20 || (tree[id_right].getRightChild() == -2 &&
				    			   tree[id_right].getRightChild() == -2) )
   	    {
	 	ptr_result++;
		result_stack[ptr_result] = id_right;
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
	else if( visit == 2)
	{
	    ptr_path--;
	    current_node = path_stack[ptr_path];
	    //cerr << "ptr_path = " << ptr_path<< endl;
	}
    }
    find_while_loop_time = clock() - find_while_loop_time;

    //cerr << "after while " << endl;
    int i = 0;
    vector<int> indeces;
    bool test;      	
    float radius;
    vector<int>::iterator it;
    find_for_loop_time = clock();
    for(int stack_idx=0; stack_idx<ptr_result+1; stack_idx++)
    {
	current_node = result_stack[stack_idx];
        num_primitives = tree[current_node].getNumPrimitives();

	//cerr << "num primitives =" << num_primitives << endl;
	//tree[current_node].print();

	// check that the resul is in the limits
	// if not it goes pack to the parent 
	for(int i =0; i<num_primitives; i++)
	{
	    indeces = tree[current_node].getPrimitivesIndex();
	    test = data[indeces[i]].findPosition(pt);      	
	    //cerr << "test =" << test << endl;
	    if(test == true)
	    {
                radius = data[indeces[i]].getRadius(pt);
		it = find(result.begin(), result.end(), indeces[i]);
		if(radius <= r && it == result.end())
 	           result.push_back(indeces[i]);
	    }
	}
    }
    find_for_loop_time = clock() - find_for_loop_time;
    //cerr << "find_while_loop_time = " << find_while_loop_time << endl;
    //cerr << "find_for_loop_time = " << find_for_loop_time << endl;
    // check for radius and grow
}

//*************************************************************
// This is the traditional find slow for loop 

// fro comparaison purposes
//**************************************************************
void 		KDTREESpliter::traditionalFind(float * pt, float r, vector<int> & result)
{
    for(int i=0; i < getDataSize(); i++)
    {
	bool test = data[i].findPosition(pt);      	
	//cerr << "test =" << test << endl;
	if(test == true)
	{
	    float radius = data[i].getRadius(pt);
	    if(radius <= r)
		 result.push_back(i);
	}
    }
}

//*************************************************************
// testing while I go
//*************************************************************
void 		KDTREESpliter::test()
{
    clock_t build_time, search_time;
    float build_time_sec = -10, search_time_sec = -10;
    static float pt[4] = {.99, .99, .99, 4.5};
    vector<int> result_0;
    vector<int> result_1;

    build_time = clock();
    build();

    build_time = clock() - build_time;
    build_time_sec = ((float)build_time)/ CLOCKS_PER_SEC;

    search_time = clock();
    //cerr << "debugging line -----" << endl;
    findNode(pt, .05, result_0);
    search_time = clock() - search_time;
    search_time_sec = ((float)search_time)/CLOCKS_PER_SEC;

    traditionalFind(pt, .05, result_1);
    cout << "*********************KDTREE Results*********************" << endl;
    cout << "Build time = " << build_time_sec << "		Search time =" << search_time_sec << endl;
    cout << "size of result 0 = " << result_0.size() <<endl;
    cout << "size of result 1 = " << result_1.size() <<endl;
/*
    int tree_size = 0;
    getTreeSizeInByte(tree[0], tree_size);
    int size_of_data = sizeof(data) + getSizeInByte();
    int size_of_tree = sizeof(tree) + tree_size; 
    cout << "size of data in byte = " << size_of_data<< endl;
    cout << "size of tree in byte = " <<size_of_tree<< endl;

    // save in files
    fstream file;
    file.open("KDTfile", fstream::in | fstream::out | fstream::app);
    file <<  size_of_data << "\t";
    file <<  size_of_tree << "\t";
    file <<  build_time << "\t";
    file <<  search_time << "\n";
    file.close();
    
*/
    for(int i=0; i < result_0.size(); i++)
    	cout << result_0[i] << ", ";
    cout<< endl;
    for(int i=0; i < result_1.size(); i++)
    	cout << result_1[i] << ", ";
    cout<< endl;
}
