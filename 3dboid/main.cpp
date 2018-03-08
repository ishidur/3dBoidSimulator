// main.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// Created by Ryota Ishidu, Morishita Lab.

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "Grid.h"
#include "ppl.h"
#include "BaseBoid.h"
#include "Direction.h"
#include "Block.h"
#include "Eigen/Core"

int tim = 0; //tim
bool is_press = false;
double mouse_x = 0.0;
double mouse_y = 0.0;

GLfloat light0_pos[] = {0.0, 0.0, BOUNDARY, 1.0};
GLfloat light1_pos[] = {0.0, 0.0, BOUNDARY, 1.0};

GLfloat green[] = {0.0, 1.0, 0.0, 1.0};
GLfloat red[] = {0.8, 0.2, 0.2, 1.0};
GLfloat blue[] = {0.0, 0.0, 1.0, 0.6};
GLfloat white[] = {0.5, 0.5, 0.5, 0.4};

double calc_dist(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

//[x][y][z]
Grid grids[GRID_NO + NEAR_GRID_NO * 2][GRID_NO + NEAR_GRID_NO * 2][GRID_NO + NEAR_GRID_NO * 2];

std::vector<Block> blocks;

//this function needs grids
std::vector<int> get_around_grid_boids(int id, int grid_x, int grid_y, int grid_z)
{
	std::vector<int> indexes = grids[grid_x][grid_y][grid_z].boid_indexes;
	for (int i = -NEAR_GRID_NO; i <= NEAR_GRID_NO; ++i)
	{
		for (int j = -NEAR_GRID_NO + abs(i); j <= NEAR_GRID_NO - abs(i); ++j)
		{
			for (int k = -NEAR_GRID_NO + abs(i) + abs(j); k <= NEAR_GRID_NO - abs(i) - abs(j); ++k)
			{
				indexes.insert(indexes.end(), grids[grid_x + i][grid_y + j][grid_z + k].boid_indexes.begin(),
				               grids[grid_x + i][grid_y + j][grid_z + k].boid_indexes.end());
			}
		}
	}
	auto result = remove(indexes.begin(), indexes.end(), id);
	auto result2 = unique(indexes.begin(), result);
	indexes.erase(result2, indexes.end());
	return indexes;
}

std::vector<BaseBoid> boids;

double degree_to_radian(double deg) { return deg * M_PI / 180.0; }

//this needs for Biod::isVisible
double view_angle = degree_to_radian(THETA_1) / 2.0;

BaseBoid update_speed_and_angle(BaseBoid& boid)
{
	Eigen::Vector3d q1, q2, q3, q4;
	q1 = q2 = q3 = q4 = Eigen::Vector3d::Zero();
	int n1, n2, n3, n4;
	n1 = n2 = n3 = n4 = 0;
	auto indexes = get_around_grid_boids(boid.id, boid.grid_x, boid.grid_y, boid.grid_z);
	/*loop starts here*/
	for (auto i : indexes)
	{
		double dist = calc_dist(boid.x, boid.y, boid.z, boids[i].x, boids[i].y, boids[i].z);
		if (boid.is_visible(boids[i].x, boids[i].y, boids[i].z, view_angle))
		{
			/*boidが見える範囲内にいる*/
			if (dist - 2.0 * BOID_SIZE < R_1)
			{
				/*rule1*/
				n1++;
				q1 += boids[i].vctr.normalized();
				if (boid.id == 0) { boids[i].set_color(1.0, 1.0, 0.0); }
			}
			if (dist - 2.0 * BOID_SIZE < R_2)
			{
				/*rule2*/
				n2++;
				q2 += Eigen::Vector3d(boids[i].x - boid.x, boids[i].y - boid.y, boids[i].z - boid.z) / dist / dist * R_2;
				if (boid.id == 0) { boids[i].set_color(1.0, 1.0, 0.0); }
			}
			if (dist - 2.0 * BOID_SIZE < R_3)
			{
				/*rule3*/
				n3++;
				q3 += Eigen::Vector3d(boids[i].x - boid.x, boids[i].y - boid.y, boids[i].z - boid.z) / dist / dist * R_3;
				if (boid.id == 0) { boids[i].set_color(0.0, 1.0, 0.0); }
			}
		}
	}
	/*loop ends here*/
	for (int n : grids[boid.grid_x][boid.grid_y][boid.grid_z].block_indexes)
	{
		double dist = calc_dist(boid.x, boid.y, boid.z, blocks[n].x, blocks[n].y, blocks[n].z);
		if (dist - BLOCK_SIZE - BOID_SIZE <= R_4 && !blocks[n].disabled)
		{
			/*rule4*/
			n4++;
			q4 += Eigen::Vector3d(blocks[n].x - boid.x, blocks[n].y - boid.y, blocks[n].z - boid.z) / dist / dist * R_4;
		}
	}
	//	if (isPress)
	//	{
	//		double dist = calcDist(boid.x, boid.y, mouseX, mouseY);
	//		if (dist - BOID_SIZE <= R_4)
	//		{
	//			/*rule4*/
	//			n4++;
	//			q4 += Eigen::Vector3d(mouseX - boid.x, mouseY - boid.y) / dist / dist * R_4;
	//		}
	//	}
	if (n1 != 0) { q1 /= double(n1); }
	if (n2 != 0) { q2 /= double(n2); }
	if (n3 != 0) { q3 /= double(n3); }
	if (n4 != 0) { q4 /= double(n4); }

	/*wall repel*/
	Eigen::Vector3d wallRepel = Eigen::Vector3d::Zero();
	double wall = BOUNDARY - WALL_SIZE;
	double bound = wall - BOID_SIZE - R_4;
	if (boid.x >= bound) { wallRepel.x() = 1.0 / (wall - boid.x); }
	else if (boid.x <= -bound) { wallRepel.x() = -1.0 / (wall + boid.x); }
	if (boid.y >= bound) { wallRepel.y() = 1.0 / (wall - boid.y); }
	else if (boid.y <= -bound) { wallRepel.y() = -1.0 / (wall + boid.y); }
	if (boid.z >= bound) { wallRepel.z() = 1.0 / (wall - boid.z); }
	else if (boid.z <= -bound) { wallRepel.z() = -1.0 / (wall + boid.z); }
	Eigen::Vector3d V = ALPHA_1 * q1.normalized() + ALPHA_2 * q2 - ALPHA_3 * q3 - ALPHA_4 * q4 + ALPHA_5 * boid
	                                                                                                       .vctr.
	                                                                                                       normalized() -
		REPEL_WALL_WEIGHT * wallRepel;
	Direction dir = Direction(V);
	boid.angle_y = dir.angle_y;
	boid.angle_z = dir.angle_z;
	boid.speed = BETA * log(V.norm() + 1.0);
	boid.vctr = boid.speed * dir.vector;
	return boid;
}

//this function needs grids
void create_grids()
{
	double width = 2.0 * BOUNDARY / GRID_NO;
	double left;
	double front;
	double top = -BOUNDARY - double(NEAR_GRID_NO) * width;
	for (int i = 0; i < GRID_NO + NEAR_GRID_NO * 2; ++i)
	{
		left = -BOUNDARY - double(NEAR_GRID_NO) * width;
		for (int j = 0; j < GRID_NO + NEAR_GRID_NO * 2; ++j)
		{
			front = -BOUNDARY - double(NEAR_GRID_NO) * width;
			for (int k = 0; k < GRID_NO + NEAR_GRID_NO * 2; ++k)
			{
				grids[i][j][k] = Grid(top, top + width, left, left + width, front, front + width);
				front += width;
			}
			left += width;
		}
		top += width;
	}
}

//this function needs grids, boids: 非効率かも
void update_grids()
{
	Concurrency::parallel_for(NEAR_GRID_NO, GRID_NO + NEAR_GRID_NO, 1, [](int i)
	{
		for (int j = NEAR_GRID_NO; j <= GRID_NO + NEAR_GRID_NO; j++)
		{
			for (int k = NEAR_GRID_NO; k <= GRID_NO + NEAR_GRID_NO; ++k)
			{
				std::vector<int> indexes = grids[i][j][k].boid_indexes;
				for (auto n : indexes)
				{
					if (boids[n].grid_x != i || boids[n].grid_y != j || boids[n].grid_z != k)
					{
						grids[i][j][k].delete_boid_by_index(n);
					}
				}
			}
		}
	});
}

//this function needs grids, boids
void find_grid(int index, double x, double y, double z)
{
	double width = 2.0 * BOUNDARY / GRID_NO;
	int gridx = int(ceil((BOUNDARY + x) / width)) - 1 + NEAR_GRID_NO;
	int gridy = int(ceil((BOUNDARY + y) / width)) - 1 + NEAR_GRID_NO;
	int gridz = int(ceil((BOUNDARY + z) / width)) - 1 + NEAR_GRID_NO;
	grids[gridx][gridy][gridz].add_boid_by_index(index);
	boids[index].grid_x = gridx;
	boids[index].grid_y = gridy;
	boids[index].grid_z = gridz;
}

//BUG: whaaaat
void where_block(int index, double x, double y, double z)
{
	double width = 2.0 * BOUNDARY / GRID_NO;
	int gridx = int(ceil((BOUNDARY + x) / width)) - 1 + NEAR_GRID_NO;
	int gridy = int(ceil((BOUNDARY + y) / width)) - 1 + NEAR_GRID_NO;
	int gridz = int(ceil((BOUNDARY + z) / width)) - 1 + NEAR_GRID_NO;
	for (int i = -NEAR_GRID_NO; i <= NEAR_GRID_NO; ++i)
	{
		for (int j = -NEAR_GRID_NO + abs(i); j <= NEAR_GRID_NO - abs(i); ++j)
		{
			for (int k = -NEAR_GRID_NO + abs(i) + abs(j); k <= NEAR_GRID_NO - abs(i) - abs(j); ++k)
			{
				grids[gridx + i][gridy + j][gridz + k].add_block_by_index(index);
			}
		}
	}
}

void remove_block(int index, double x, double y, double z)
{
	double width = 2.0 * BOUNDARY / GRID_NO;
	int gridx = int(ceil((BOUNDARY + x) / width)) - 1 + NEAR_GRID_NO;
	int gridy = int(ceil((BOUNDARY + y) / width)) - 1 + NEAR_GRID_NO;
	int gridz = int(ceil((BOUNDARY + z) / width)) - 1 + NEAR_GRID_NO;

	for (int i = -NEAR_GRID_NO; i <= NEAR_GRID_NO; ++i)
	{
		for (int j = -NEAR_GRID_NO + abs(i); j <= NEAR_GRID_NO - abs(i); ++j)
		{
			for (int k = -NEAR_GRID_NO + abs(i) + abs(j); k <= NEAR_GRID_NO - abs(i) - abs(j); ++k)
			{
				grids[gridx + i][gridy + j][gridz + k].delete_block_by_index(index);
			}
		}
	}
	blocks[index].set_disabled();
}

void remove_all_blocks()
{
	for (int i = 0; i < GRID_NO + NEAR_GRID_NO * 2; ++i)
	{
		for (int j = 0; j < GRID_NO + NEAR_GRID_NO * 2; ++j)
		{
			for (int k = 0; k < GRID_NO + NEAR_GRID_NO * 2; ++k) { grids[i][j][k].delete_all_blocks(); }
		}
	}
	blocks.clear();
}

int find_duplicate_block(double x, double y, double z)
{
	double width = 2.0 * BOUNDARY / GRID_NO;
	int gridx = int(ceil((BOUNDARY + x) / width)) - 1 + NEAR_GRID_NO;
	int gridy = int(ceil((BOUNDARY + y) / width)) - 1 + NEAR_GRID_NO;
	int gridz = int(ceil((BOUNDARY + z) / width)) - 1 + NEAR_GRID_NO;
	for (auto i : grids[gridx][gridy][gridz].block_indexes)
	{
		double dist = calc_dist(x, y, z, blocks[i].x, blocks[i].y, blocks[i].z);
		if (dist <= 2.0 * BLOCK_SIZE && !blocks[i].disabled) { return i; }
	}
	return -1;
}

void draw_wall()
{
	double bound = BOUNDARY;
	GLfloat color[] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
	glBegin(GL_POLYGON);
	glColor3d(1.0, 0.0, 0.0);
	glVertex3d(bound, bound, bound);
	glVertex3d(bound, -bound, bound);
	glVertex3d(bound, -bound, -bound);
	glVertex3d(bound, bound, -bound);
	glEnd();
	glBegin(GL_POLYGON);
	glColor3d(0.0, 1.0, 0.0);
	glVertex3d(-bound, bound, bound);
	glVertex3d(-bound, bound, -bound);
	glVertex3d(-bound, -bound, -bound);
	glVertex3d(-bound, -bound, bound);
	glEnd();
	glBegin(GL_POLYGON);
	glColor3d(0.0, 0.0, 1.0);
	glVertex3d(bound, bound, bound);
	glVertex3d(bound, bound, -bound);
	glVertex3d(-bound, bound, -bound);
	glVertex3d(-bound, bound, bound);
	glEnd();
	glBegin(GL_POLYGON);
	glColor3d(1.0, 1.0, 0.0);
	glVertex3d(bound, -bound, bound);
	glVertex3d(-bound, -bound, bound);
	glVertex3d(-bound, -bound, -bound);
	glVertex3d(bound, -bound, -bound);
	glEnd();
	glBegin(GL_POLYGON);
	glColor3d(1.0, 0.0, 1.0);
	glVertex3d(bound, bound, -bound);
	glVertex3d(bound, -bound, -bound);
	glVertex3d(-bound, -bound, -bound);
	glVertex3d(-bound, bound, -bound);
	glEnd();
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//	glClear(GL_COLOR_BUFFER_BIT);

	//	/* 光源の位置設定 */
	glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);

	draw_wall();
	for (auto boid : boids) { boid.draw_base_boid(); }
	for (auto block : blocks)
	{
		if (!block.disabled) { block.draw_block(); }
	}
	glFlush();
}

void resize(int w, int h)
{
	glViewport(0, 0, w, h);

	/* 透視変換行列の設定 */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(120.0, double(w) / double(h), 0.0001, double(w) / WINDOW_SIZE * BOUNDARY * 3.0);

	/* モデルビュー変換行列の設定 */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, double(w) / WINDOW_SIZE * BOUNDARY * 1.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

/*
void mouse(int button, int state, int x, int y)
{
	double pos_x = BOUNDARY * (double(x) - WINDOW_SIZE / 2.0) / double(WINDOW_SIZE / 2.0);
	double pos_y = -BOUNDARY * (double(y) - WINDOW_SIZE / 2.0) / double(WINDOW_SIZE / 2.0);
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		int index = findDuplicateBlock(pos_x, pos_y);
		if (index != -1)
		{
			std::cout << "exist" << index << std::endl;
			removeBlock(index, pos_x, pos_y);
		}
		else
		{
			blocks.push_back(Block(pos_x, pos_y, BLOCK_SIZE));
			whereBlock(blocks.size() - 1, blocks[blocks.size() - 1].x, blocks[blocks.size() - 1].y);
		}
	}
	if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			std::cout << "pressing" << std::endl;
			mouseX = pos_x;
			mouseY = pos_y;
			isPress = true;
		}
		else
		{
			isPress = false;
		}
	}
}

void motion(int x, int y)
{
	if (isPress)
	{
		double pos_x = BOUNDARY * (double(x) - WINDOW_SIZE / 2.0) / double(WINDOW_SIZE / 2.0);
		double pos_y = -BOUNDARY * (double(y) - WINDOW_SIZE / 2.0) / double(WINDOW_SIZE / 2.0);
		mouseX = pos_x;
		mouseY = pos_y;
	}
}
*/

void key(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'r':
		{
			std::cout << "refresh" << std::endl;
			remove_all_blocks();
		}
	case 'b':
		{
			remove_all_blocks();
			for (int i = 0; i < BLOCK_NO; ++i)
			{
				blocks.push_back(Block((double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
				                       (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
				                       (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
				                       BLOCK_SIZE));
				where_block(i, blocks[i].x, blocks[i].y, blocks[i].z);
			}
		}
	}
}

void find_grids() { for (int i = 0; i < boids.size(); ++i) { find_grid(i, boids[i].x, boids[i].y, boids[i].z); } }

void timer(int value)
{
	//	if (tim % 10 == 0)
	//	{
	//		cout << tim / 10 << endl;
	//	}
	clock_t start = clock(); // start

	Concurrency::parallel_for<int>(0, boids.size(), 1, [](int i)
	{
		boids[i].update_position();
		if (i != 0) { boids[i].set_color(1.0, 1.0, 1.0); }
	});
	find_grids();
	update_grids();
	Concurrency::parallel_for<int>(0, boids.size(), 1, [](int i)
	{
		//boid速度ベクトルの計算部分
		boids[i] = update_speed_and_angle(boids[i]);
	});
	glutPostRedisplay();
	tim++;
	clock_t end = clock(); // end
	std::cout << "duration = " << float(end - start) / CLOCKS_PER_SEC << "sec\r" << std::flush;
	glutTimerFunc(FLAME_RATE, timer, tim);
}


void init()
{
	glClearColor(0.0, 0.0, 0.0, 0.5);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, blue);
	glLightfv(GL_LIGHT0, GL_SPECULAR, blue);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT1, GL_SPECULAR, white);
}

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
	//	glutInitDisplayMode(GLUT_RGBA);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);

	glutCreateWindow(argv[0]);
	//	glutMouseFunc(mouse);
	glutKeyboardFunc(key);
	init();
	create_grids();

	for (int i = 0; i < BOIDS_NO; i++)
	{
		boids.push_back(BaseBoid((double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - WALL_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
		                         (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - WALL_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
		                         (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - WALL_SIZE - BOID_SIZE) * 2.0 / RAND_MAX,
		                         (double(rand()) / RAND_MAX) * 2.0 * M_PI, (double(rand()) / RAND_MAX) * 2.0 * M_PI,
		                         BOID_SPEED, i));
		find_grid(i, boids[i].x, boids[i].y, boids[i].z);
		if (i == 0) { boids[i].set_color(1.0, 0.0, 0.0); }
	}
	for (int i = 0; i < BLOCK_NO; ++i)
	{
		blocks.push_back(Block((double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE) * 2.0 / RAND_MAX,
		                       (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE) * 2.0 / RAND_MAX,
		                       (double(rand()) - RAND_MAX / 2.0) * (BOUNDARY - BLOCK_SIZE) * 2.0 / RAND_MAX, BLOCK_SIZE));
		where_block(i, blocks[i].x, blocks[i].y, blocks[i].z);
	}

	update_grids();
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutTimerFunc(FLAME_RATE, timer, tim);
	glutMainLoop();
	return 0;
}
