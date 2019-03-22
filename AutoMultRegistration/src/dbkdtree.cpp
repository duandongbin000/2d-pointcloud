//#pragma once
#include "dbtype.h"
#include "dbkdtree.h"

#include <iostream>  
#include <algorithm>  
#include <stack>  
#include <math.h>  

using namespace std;
/*function of this program: build a 2d tree using the input training data
the input is exm_set which contains a list of tuples (x,y)
the output is a 2d tree pointer*/
//struct data
//{
//	double x = 0;
//	double y = 0;
//};

bool cmp1(db::Point2f& a, db::Point2f& b) {
	return a.x < b.x;
}
bool cmp2(db::Point2f& a, db::Point2f& b) {
	return a.y < b.y;
}
bool equal(db::Point2f& a, db::Point2f& b) {
	if (a.x == b.x && a.y == b.y)
	{
		return true;
	}
	else {
		return false;
	}
}

void ChooseSplit(db::Point2f exm_set[], int size, int &split, db::Point2f &SplitChoice) {
	/*compute the variance on every dimension. Set split as the dismension that have the biggest
	variance. Then choose the instance which is the median on this split dimension.*/
	/*compute variance on the x,y dimension. DX=EX^2-(EX)^2*/
	double tmp1, tmp2;
	tmp1 = tmp2 = 0;
	for (int i = 0; i < size; ++i)
	{
		tmp1 += 1.0 / (double)size * exm_set[i].x * exm_set[i].x;
		tmp2 += 1.0 / (double)size * exm_set[i].x;
	}
	double v1 = tmp1 - tmp2 * tmp2;  //compute variance on the x dimension  
	tmp1 = tmp2 = 0;
	for (int i = 0; i < size; ++i)
	{
		tmp1 += 1.0 / (double)size * exm_set[i].y * exm_set[i].y;
		tmp2 += 1.0 / (double)size * exm_set[i].y;
	}
	double v2 = tmp1 - tmp2 * tmp2;  //compute variance on the y dimension  
	split = v1 > v2 ? 0 : 1; //set the split dimension  
	if (split == 0)
	{
		sort(exm_set, exm_set + size, cmp1);
	}
	else {
		sort(exm_set, exm_set + size, cmp2);
	}
	//set the split point value  
	SplitChoice.x = exm_set[size / 2].x;
	SplitChoice.y = exm_set[size / 2].y;
}

Tnode* build_kdtree(db::Point2f exm_set[], int size, Tnode* T) 
{
	//call function ChooseSplit to choose the split dimension and split point  
	if (size == 0) {
		return NULL;
	}
	else 
	{
		int split;
		db::Point2f dom_elt;
		ChooseSplit(exm_set, size, split, dom_elt);
		db::Point2f *exm_set_right = new db::Point2f[size]; //[100];
		db::Point2f *exm_set_left= new db::Point2f[size];  //[100];

		int sizeleft, sizeright;
		sizeleft = sizeright = 0;
		if (split == 0)
		{
			for (int i = 0; i < size; ++i)
			{
				if (!equal(exm_set[i], dom_elt) && exm_set[i].x <= dom_elt.x)
				{
					exm_set_left[sizeleft].x = exm_set[i].x;
					exm_set_left[sizeleft].y = exm_set[i].y;
					sizeleft++;
				}
				else if (!equal(exm_set[i], dom_elt) && exm_set[i].x > dom_elt.x)
				{
					exm_set_right[sizeright].x = exm_set[i].x;
					exm_set_right[sizeright].y = exm_set[i].y;
					sizeright++;
				}
			}
		}
		else {
			for (int i = 0; i < size; ++i)
			{
				if (!equal(exm_set[i], dom_elt) && exm_set[i].y <= dom_elt.y)
				{
					exm_set_left[sizeleft].x = exm_set[i].x;
					exm_set_left[sizeleft].y = exm_set[i].y;
					sizeleft++;
				}
				else if (!equal(exm_set[i], dom_elt) && exm_set[i].y > dom_elt.y)
				{
					exm_set_right[sizeright].x = exm_set[i].x;
					exm_set_right[sizeright].y = exm_set[i].y;
					sizeright++;
				}
			}
		}
		T = new Tnode;
		T->dom_elt.x = dom_elt.x;
		T->dom_elt.y = dom_elt.y;
		T->split = split;
		T->left = build_kdtree(exm_set_left, sizeleft, T->left);
		T->right = build_kdtree(exm_set_right, sizeright, T->right);
		return T;
	}
}

Tnode* build_kdtree(std::vector<db::Point2f>& point_set)
{
	int size = point_set.size();
	db::Point2f *exm_set=new db::Point2f[size];
	for (size_t i = 0; i < point_set.size(); i++)
	{
		exm_set[i].x = point_set[i].x;
		exm_set[i].y = point_set[i].y;
	}

	struct Tnode * root = NULL;
	root = build_kdtree(exm_set, size, root);
	return root;
}

double Distance(db::Point2f& a, db::Point2f& b) {
	double tmp = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
	return sqrt(tmp);
}

void searchNearest(Tnode * Kd, db::Point2f& target, db::Point2f &nearestpoint, double & distance) {
	//1. ���Kd�ǿյģ�����distΪ����󷵻�  
	//2. ��������ֱ��Ҷ�ӽ��  
	stack<Tnode*> search_path;
	Tnode* pSearch = Kd;
	db::Point2f nearest;
	double dist;
	while (pSearch != NULL)
	{
		//pSearch���뵽search_path��;  
		search_path.push(pSearch);
		if (pSearch->split == 0)
		{
			if (target.x <= pSearch->dom_elt.x) /* ���С�ھͽ��������� */
			{
				pSearch = pSearch->left;
			}
			else
			{
				pSearch = pSearch->right;
			}
		}
		else {
			if (target.y <= pSearch->dom_elt.y) /* ���С�ھͽ��������� */
			{
				pSearch = pSearch->left;
			}
			else
			{
				pSearch = pSearch->right;
			}
		}
	}
	//ȡ��search_path���һ������nearest  
	nearest.x = search_path.top()->dom_elt.x;
	nearest.y = search_path.top()->dom_elt.y;
	search_path.pop();
	dist = Distance(nearest, target);
	//3. ��������·��  
	Tnode* pBack;
	while (search_path.size() != 0)
	{
		//ȡ��search_path���һ����㸳��pBack  
		pBack = search_path.top();
		search_path.pop();
		if (pBack->left == NULL && pBack->right == NULL) /* ���pBackΪҶ�ӽ�� */
		{
			if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
			{
				nearest = pBack->dom_elt;
				dist = Distance(pBack->dom_elt, target);
			}
		}
		else
		{
			int s = pBack->split;
			if (s == 0)
			{
				if (fabs(pBack->dom_elt.x - target.x) < dist) /* �����targetΪ���ĵ�Բ������򣩣��뾶Ϊdist��Բ��ָƽ���ཻ�� ��ô��Ҫ������һ�ߵ��ӿռ�ȥ���� */
				{
					if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
					{
						nearest = pBack->dom_elt;
						dist = Distance(pBack->dom_elt, target);
					}
					if (target.x <= pBack->dom_elt.x) /* ���targetλ��pBack�����ӿռ䣬��ô��Ҫ�������ӿռ�ȥ���� */
						pSearch = pBack->right;
					else
						pSearch = pBack->left; /* ���targetλ��pBack�����ӿռ䣬��ô��Ҫ�������ӿռ�ȥ���� */
					if (pSearch != NULL)
						//pSearch���뵽search_path��  
						search_path.push(pSearch);
				}
			}
			else {
				if (fabs(pBack->dom_elt.y - target.y) < dist) /* �����targetΪ���ĵ�Բ������򣩣��뾶Ϊdist��Բ��ָƽ���ཻ�� ��ô��Ҫ������һ�ߵ��ӿռ�ȥ���� */
				{
					if (Distance(nearest, target) > Distance(pBack->dom_elt, target))
					{
						nearest = pBack->dom_elt;
						dist = Distance(pBack->dom_elt, target);
					}
					if (target.y <= pBack->dom_elt.y) /* ���targetλ��pBack�����ӿռ䣬��ô��Ҫ�������ӿռ�ȥ���� */
						pSearch = pBack->right;
					else
						pSearch = pBack->left; /* ���targetλ��pBack�����ӿռ䣬��ô��Ҫ�������ӿռ�ȥ���� */
					if (pSearch != NULL)
						// pSearch���뵽search_path��  
						search_path.push(pSearch);
				}
			}
		}
	}
	nearestpoint.x = nearest.x;
	nearestpoint.y = nearest.y;
	distance = dist;
}


int main_test(){
	db::Point2f exm_set[100]; //assume the max training set size is 100  
	double x, y;
	int id = 0;
	cout << "Please input the training data in the form x y. One instance per line. Enter -1 -1 to stop." << endl;
	while (cin >> x >> y) {
		if (x == -1)
		{
			break;
		}
		else {
			exm_set[id].x = x;
			exm_set[id].y = y;
			id++;
		}
	}
	struct Tnode * root = NULL;
	root = build_kdtree(exm_set, id, root);
	db::Point2f nearestpoint;
	double distance;
	db::Point2f target;
	cout << "Enter search point" << endl;
	while (cin >> target.x >> target.y)
	{
		searchNearest(root, target, nearestpoint, distance);
		cout << "The nearest distance is " << distance << ",and the nearest point is " << nearestpoint.x << "," << nearestpoint.y << endl;
		cout << "Enter search point" << endl;
	}
	return 0;
}


