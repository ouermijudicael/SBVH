#include <iostream>
#include <math.h>
#include <stdlib.h>
using std::cerr;
using std::endl;

int
float_compare(const void *A, const void *B)
{
    float fA = *((float *)A);
    float fB = *((float *)B);
    if (fA < fB)
        return -1;
    if (fB < fA)
        return +1;
    return 0;
}

void
EvaluatePivot(float *segments, int numSegments, float pivot, 
              int &numLeft, int &numSpan, int &numRight)
{
    numLeft  = 0;
    numSpan  = 0;
    numRight = 0;
    for (int i = 0 ; i < numSegments ; i++)
    {
        if (segments[2*i] <= pivot && pivot <= segments[2*i+1])
            numSpan++;
        else if (segments[2*i+1] < pivot)
            numLeft++;
        else if (segments[2*i] > pivot)
            numRight++;
        else
        {
            cerr << "Should not happen!!!" << endl;
        }
    }
}

int main()
{
     int numSegments = 100000;
     float *segments = new float[numSegments*2];
     for (int i = 0 ; i < numSegments ; i++)
     {
          segments[2*i] = (rand()%10000)/10000.0;
          segments[2*i+1] = (rand()%10000)/10000.0;
          if (segments[2*i+1] < segments[2*i])
          {
               float tmp = segments[2*i+1];
               segments[2*i+1] = segments[2*i];
               segments[2*i] = tmp;
          }
     }
   
     int list_size = 1000;
     float *possible_pivots = new float[list_size];
     for (int i = 0 ; i < list_size ; i++)
          possible_pivots[i] = (rand()%10000)/10000.0;
     qsort((void *)possible_pivots, list_size, sizeof(float), float_compare);

     cerr << "Pivots are " << possible_pivots[0] << endl;
     cerr << "Pivots are " << possible_pivots[500] << endl;
     cerr << "Pivots are " << possible_pivots[999] << endl;

     int numLeft, numSpan, numRight;

     int min = 0;
     int max = list_size;
     int guess = (min+max)/2;
     while ((max - min) > 1)
     {
         EvaluatePivot(segments, numSegments, possible_pivots[guess], 
                       numLeft, numSpan, numRight);
         cerr << "Guess = " <<  guess <<" (" << possible_pivots[guess] << "), NL = " << numLeft << ", NS = " << numSpan << ", NR = " << numRight << endl;
         if (numLeft == numRight)
         {
             cerr << "found perfect match" << endl;
             exit(0);
         }
         if (numLeft > numRight)
             max = guess;
         else
             min = guess;
         guess = (min+max)/2;
     }
     
}

