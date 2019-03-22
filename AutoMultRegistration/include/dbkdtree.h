#pragma once

#include "dbtype.h"

#include <iostream>  
#include <algorithm>  
#include <stack>  
#include <math.h>  

using namespace std;

struct Tnode
{
	struct db::Point2f dom_elt;
	int split;
	struct Tnode * left;
	struct Tnode * right;
};

void ChooseSplit(db::Point2f exm_set[], int size, int &split, db::Point2f &SplitChoice);
Tnode* build_kdtree(db::Point2f exm_set[], int size, Tnode* T);
Tnode* build_kdtree(std::vector<db::Point2f>& point_set);
double Distance(db::Point2f& a, db::Point2f& b);
void searchNearest(Tnode * Kd, db::Point2f& target, db::Point2f &nearestpoint, double & distance);