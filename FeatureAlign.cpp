///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.h -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
// (modified for CSE576 Spring 2005)
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include <math.h>

/******************* TO DO *********************
* alignPair:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*               *NOTE* Each match in 'matches' contains two feature ids of matching features, id1 (in f1) and id2 (in f2).
*               These ids are 1-based indices into the feature arrays,
*               so you access the appropriate features as f1[id1-1] and f2[id2-1].
*		m: motion model
*		f: focal length
*		nRANSAC: number of RANSAC iterations
*		RANSACthresh: RANSAC distance threshold
*		M: transformation matrix (output)
*	OUTPUT:
*		repeat for nRANSAC iterations:
*			choose a minimal set of feature matches
*			estimate the transformation implied by these matches
*			count the number of inliers
*		for the transformation with the maximum number of inliers,
*		compute the least squares motion estimate using the inliers,
*		and store it in M
*/
int alignPair(const FeatureSet &f1, const FeatureSet &f2,
              const vector<FeatureMatch> &matches, MotionModel m, float f,
              int nRANSAC, double RANSACthresh, CTransform3x3& M)
{
    // BEGIN TODO
    // write this entire method
	vector<int> optimalInliers;
	int maxInlierCount=0;
	for(int i=0;i<nRANSAC;i++)
	{
		int randMatch=rand()%matches.size();
		FeatureMatch fMatch=matches[randMatch];
		double xTrans=f1[fMatch.id1-1].x-f2[fMatch.id2-1].x;
		double yTrans=f1[fMatch.id1-1].y-f2[fMatch.id2-1].y;
		CTransform3x3 transM=CTransform3x3::Translation(xTrans, yTrans);
		vector<int> inliers;
		int inlierCount=countInliers(f1, f2, matches, m, f, transM, RANSACthresh, inliers);
		if(inlierCount>maxInlierCount){
			maxInlierCount=inlierCount;
			optimalInliers.clear();
			optimalInliers.insert(optimalInliers.end(), inliers.begin(), inliers.end());
		}
	}
	leastSquaresFit(f1, f2, matches, m, f, optimalInliers, M);
    // END TODO
    return 0;
}

/******************* TO DO *********************
* countInliers:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*               *NOTE* Each match contains two feature ids of matching features, id1 (in f1) and id2 (in f2).
*               These ids are 1-based indices into the feature arrays,
*               so you access the appropriate features as f1[id1-1] and f2[id2-1].
*		m: motion model
*		f: focal length
*		M: transformation matrix
*		RANSACthresh: RANSAC distance threshold
*		inliers: inlier feature IDs
*	OUTPUT:
*		transform the matched features in f1 by M
*
*		count the number of matching features for which the transformed
*		feature f1[id1-1] is within SSD distance RANSACthresh of its match
*		f2[id2-1]
*
*		store the indices of these matches in inliers
*
*		
*/
int countInliers(const FeatureSet &f1, const FeatureSet &f2,
                 const vector<FeatureMatch> &matches, MotionModel m, float f,
                 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
    inliers.clear();
    int count = 0;

    for (unsigned int i=0; i<(int) matches.size(); i++) {
        // BEGIN TODO
        // determine if the ith matched feature f1[id1-1], when transformed by M,
        // is within RANSACthresh of its match in f2
        //
        // if so, increment count and append i to inliers
        //
        // *NOTE* Each match contains two feature ids of matching features, id1 and id2.
        //        These ids are 1-based indices into the feature arrays,
        //        so you access the appropriate features as f1[id1-1] and f2[id2-1].
		FeatureMatch fMatch=matches[i];
		Feature feature1=f1[fMatch.id1-1];
		Feature feature2=f2[fMatch.id2-1];
		CVector3 p1;
		CVector3 p2=M*p1;
		if(p2[2]!=0){
			p2[0]=p2[0]/p2[2];p2[1]=p2[1]/p2[2];
			double dist=sqrt(pow(p2[0]-feature2.x, 2)+pow(p2[1]-feature2.y, 2));
			if(dist<=RANSACthresh){
				count++;
				inliers.push_back(i);
			}
		}
        // END TODO
    }

    return count;
}

/******************* TO DO *********************
* leastSquaresFit:
*	INPUT:
*		f1, f2: source feature sets
*		matches: correspondences between f1 and f2
*		m: motion model
*		f: focal length
*		inliers: inlier match indices (indexes into 'matches' array)
*		M: transformation matrix (output)
*	OUTPUT:
*		compute the transformation from f1 to f2 using only the inliers
*		and return it in M
*/
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
                    const vector<FeatureMatch> &matches, MotionModel m, float f,
                    const vector<int> &inliers, CTransform3x3& M)
{
    // for project 2, the transformation is a translation and
    // only has two degrees of freedom
    //
    // therefore, we simply compute the average translation vector
    // between the feature in f1 and its match in f2 for all inliers
    double u = 0;
    double v = 0;

    for (int i=0; i<inliers.size(); i++) {
        double xTrans, yTrans;

        // BEGIN TODO
        // compute the translation implied by the ith inlier match
        // and store it in (xTrans,yTrans)
		FeatureMatch fMatch=matches[inliers[i]];
		xTrans=f1[fMatch.id1-1].x-f2[fMatch.id2-1].x;
		yTrans=f1[fMatch.id1-1].y-f2[fMatch.id2-1].y;
        // END TODO

        u += xTrans;
        v += yTrans;
    }

    u /= inliers.size();
    v /= inliers.size();

    M[0][0] = 1;
    M[0][1] = 0;
    M[0][2] = u;
    M[1][0] = 0;
    M[1][1] = 1;
    M[1][2] = v;
    M[2][0] = 0;
    M[2][1] = 0;
    M[2][2] = 1;

    return 0;
}
