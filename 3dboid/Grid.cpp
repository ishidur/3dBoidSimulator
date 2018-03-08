#include "stdafx.h"
#include "Grid.h"
#include <vector>
#include <algorithm>

Grid::Grid(double _top, double _bottom, double _left, double _right, double _front, double _back)
{
	left = _left;
	right = _right;
	top = _top;
	bottom = _bottom;
	front = _front;
	back = _back;
}

void Grid::add_boid_by_index(int index)
{
	boid_indexes.push_back(index);
	sort(boid_indexes.begin(), boid_indexes.end());
}

void Grid::delete_boid_by_index(int index)
{
	if (find(boid_indexes.begin(), boid_indexes.end(), index) != boid_indexes.end())
	{
		remove(boid_indexes.begin(), boid_indexes.end(), index);
		boid_indexes.pop_back();
	}
}

bool Grid::find_boid_by_index(int index)
{
	if (find(boid_indexes.begin(), boid_indexes.end(), index) != boid_indexes.end()) { return true; }
	return false;
}

void Grid::add_block_by_index(int index)
{
	block_indexes.push_back(index);
	sort(block_indexes.begin(), block_indexes.end());
}

void Grid::delete_block_by_index(int index)
{
	if (find(block_indexes.begin(), block_indexes.end(), index) != block_indexes.end())
	{
		remove(block_indexes.begin(), block_indexes.end(), index);
		block_indexes.pop_back();
	}
}

void Grid::delete_all_blocks() { block_indexes.clear(); }
