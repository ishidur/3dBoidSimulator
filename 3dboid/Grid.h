#pragma once
#include <vector>

class Grid
{
public:
	double left;
	double right;
	double top;
	double bottom;
	double front;
	double back;
	std::vector<int> boid_indexes;
	std::vector<int> block_indexes;

	/**
	* \brief
	* \param top
	* \param bottom
	* \param left
	* \param right
	*/
	Grid(double top = 0.0, double bottom = 0.0, double left = 0.0, double right = 0.0, double front = 0.0,
	     double back = 0.0);
	/**
	* \brief
	* \param index
	*/
	void add_boid_by_index(int index);
	/**
	* \brief
	* \param index
	*/
	void delete_boid_by_index(int index);
	/**
	* \brief
	* \param index
	* \return
	*/
	bool find_boid_by_index(int index);
	/**
	* \brief
	* \param index
	*/
	void add_block_by_index(int index);
	/**
	* \brief
	* \param index
	*/
	void delete_block_by_index(int index);
	/**
	* \brief
	*/
	void delete_all_blocks();
};
