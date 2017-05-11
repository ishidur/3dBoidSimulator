#pragma once

class Block
{
private:
	double red = 0.5;
	double green = 0.5;
	double blue = 0.5;
public:
	bool disabled = false;
	double x; //center x
	double y; //center y
	double z; //center z
	double r; //radius

	/**
	 * \brief 
	 * \param x 
	 * \param y 
	 * \param z 
	 * \param r 
	 */
	Block(double x, double y, double z, double r);

	/**
	* \brief
	* \param red
	* \param green
	* \param blue
	*/
	void setColor(double red, double green, double blue);
	void drawBlock();
	void setDisabled();
};
