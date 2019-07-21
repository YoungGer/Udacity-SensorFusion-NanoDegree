/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H
#define KDTREE_H

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertHelper(Node* &node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
			node = new Node(point, id);
		else
		{
			uint ui = depth % 2;
			if (point[ui] < node->point[ui]) 
				insertHelper(node->left, depth+1, point, id);
			else
				insertHelper(node->right, depth+1, point, id);
		}

	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}



	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{
			float node_x = node->point[0];
			float node_y = node->point[1];
			float target_x = target[0];
			float target_y = target[1];
			if ((node_x >= target_x - distanceTol) && (node_x <= target_x + distanceTol) && (node_y >= target_y - distanceTol) && (node_y <= target_y + distanceTol))
			{
				float x_diff = node_x - target_x;
				float y_diff = node_y - target_y;
				float distance = sqrt(x_diff*x_diff + y_diff*y_diff);
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// recursive iterate
			int di = depth % 2;
			if (target[di] - distanceTol  < node->point[di])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if (target[di] + distanceTol  > node->point[di])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};

#endif



