/* Copyright Steve Rabin, 2012. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */
/******************************************************************************/
/*!
\file   movement_student.cpp
\author Che Wang
\par    email: che.wang\@digipen.edu
\brief  
        implemented A* algorithm 
    
*/
/******************************************************************************/
#include <Stdafx.h>

float heuris;
int type;
bool astar_analysis;
Node startnode, goalnode;
A_star a_star;
Heuristic_cal mtype;
Point point;
Node::Node()
{
	parent = NULL;
	m_gx = 0.0f;
	m_hx = 0.0f;
	m_fx = 0.0f;
	m_row = -1;
	m_col = -1;
	open = close = false;

};

Node::Node(int r, int c)
{
	m_row = r;
	m_col = c;
	m_fx = 0.0f;	
	m_gx = 0.0f;
	m_hx = 0.0f;
	parent = NULL;	
	open = close = false;
}

void Node::SetGx()
{
	//itself
	if (parent == NULL)
	{
		m_gx = 0;
		return;
	}
	if (m_row != parent->m_row && m_col != parent->m_col)
	{
		//diagonal
		m_gx = parent->m_gx + 14; //sqrt2 * 10
	}
	else 
	{	//side
		m_gx = parent->m_gx + 10;
	}

	//project3
	if (astar_analysis)
		m_gx += g_terrain.GetInfluenceMapValue(m_row, m_col) * 20;
}
void Node::SetHx(Node &curnode, Node &goalnode, Heuristic_cal type)
{
	float xdiff = (float)std::abs(goalnode.m_row - curnode.m_row) * 10;
	float ydiff = (float)std::abs(goalnode.m_col - curnode.m_col) * 10;
	switch (type)
	{
	case Manhattan:
		//xdiff + ydiff
		m_hx = xdiff + ydiff;
		break;
	case Chebyshev:
		m_hx = max(xdiff, ydiff);
		break;
	case Euclidean:
		m_hx = sqrt(xdiff * xdiff + ydiff * ydiff);
		break;
	case Octile:
		if (xdiff < ydiff)
			m_hx = xdiff * 0.4142f + ydiff;
		else
			m_hx = ydiff * 0.4142f + xdiff;		
			//m_hx = (float)(min(xdiff, ydiff) * 1.414 + max(xdiff, ydiff) - min(xdiff, ydiff));
		break;
	default:
		m_hx = 0.0f;
		break;
	}
}
void Node::SetFx(float weight)
{
	//fx = gx + hx * heuri
	m_fx = m_gx + m_hx * weight;
}



int Node::GetPositionX()
{
	return this->m_row;
}

int Node::GetPositionY()
{
	return this->m_col;
}

void Node::SetPositionXY(Node &rhs)
{
	m_row = rhs.m_row;
	m_col = rhs.m_col;
}
void Node::init()
{
	parent = NULL;
	m_gx = 0.0f;
	m_hx = 0.0f;
	m_fx = 0.0f;
	m_row = -1;
	m_col = -1;
	open = close = false;
}


A_star::A_star()
{
	foundpath = true;
	for (size_t i = 0; i < 40; i++)
		for (size_t j = 0; j < 40; j++)
			m_grid[i][j].init();
}

void A_star::ClearAllpath()
{
	m_openlist.clear();
	m_A_path.clear();
	m_rubberpath.clear();
	m_smoothpath.clear();
	m_smoothlist.clear();
	for (size_t i = 0; i < 40; i++)
		for (size_t j = 0; j < 40; j++)
			m_grid[i][j].init();
}

void Node::operator()(int &r, int &c)
{
	m_row = r;
	m_col = c;
}
void Node::operator = (Node &rhs)
{
	m_row = rhs.m_row;
	m_col = rhs.m_col;
	m_fx = rhs.m_fx;
	m_gx = rhs.m_gx;
	m_hx = rhs.m_hx;
	parent = rhs.parent;
	open = rhs.open;
	close = rhs.close;
}


bool A_star::Findpath(Node &start, Node &goal)
{

	//While (Open List is not empty)
	while (!m_openlist.empty())
	{

		//Pop cheapest node off Open List 
		Node curnode = PopCheapNode();
		m_grid[curnode.m_row][curnode.m_col].open = false;
		if(!g_blackboard.GetOccupancyMapFlag())
			g_terrain.SetColor(curnode.m_row, curnode.m_col, DEBUG_COLOR_BLUE);
		//If node is the Goal Node, then path found(RETURN ¡§found¡¨)
		if (curnode.m_row == goal.m_row && curnode.m_col == goal.m_col)
		{
			StoreInApath(curnode);
			return true;
		}

		//For (all neighboring child nodes) 
		GetNeighbours(curnode);
		//Place parent node on the Closed List
		m_grid[curnode.m_row][curnode.m_col].close = true;
		if (!g_blackboard.GetOccupancyMapFlag())
			g_terrain.SetColor(curnode.m_row, curnode.m_col, DEBUG_COLOR_YELLOW);

		//If taken too much time this frame 
		if (g_blackboard.GetSingleStep())
			return false;

	}
	//Open List empty, thus no path possible (RETURN ¡§fail¡¨)
	foundpath = false;
	return false;
}

void A_star::StoreInApath(Node node)
{
	Node *shortest = &node;
	if (m_A_path.empty())
	{
		Node temp;
		temp.SetPositionXY(*shortest);
		m_A_path.push_back(temp);
		shortest = shortest->parent;
	}
	while (shortest != NULL)
	{
		Node temp;
		temp.SetPositionXY(*shortest);
		m_A_path.push_front(temp);
		shortest = shortest->parent;
		//std::cout << "size is " << m_A_path.size() << std::endl;
	}
}

void A_star::GetNeighbours(Node &node)
{
	bool up, down, right, left;
	int row = node.GetPositionX();
	int col = node.GetPositionY();

	//bottom
	int leftNodeRow = row;
	int leftNodeCol = col - 1;
	down = AssignNeighbour(leftNodeRow, leftNodeCol, node, false);
	//Top
	leftNodeRow = row;
	leftNodeCol = col + 1;
	up = AssignNeighbour(leftNodeRow, leftNodeCol, node, false);
	//right
	leftNodeRow = row + 1;
	leftNodeCol = col;
	right = AssignNeighbour(leftNodeRow, leftNodeCol, node, false);
	//left
	leftNodeRow = row - 1;
	leftNodeCol = col;
	left = AssignNeighbour(leftNodeRow, leftNodeCol, node, false);

	bool left_up = false, left_down = false, right_up = false, right_down = false;
	if (up == true)
	{
		left_up = right_up = true;
	}
	if (down == true)
	{
		left_down = right_down = true;
	}
	if (right == true)
	{
		right_up = right_down = true;
	}
	if (left == true)
	{
		left_up = left_down = true;
	}


	//LeftTop
	leftNodeRow = row - 1;
	leftNodeCol = col + 1;
	AssignNeighbour(leftNodeRow, leftNodeCol, node, left_up);
	//RightTop
	leftNodeRow = row + 1;
	leftNodeCol = col + 1;
	AssignNeighbour(leftNodeRow, leftNodeCol, node, right_up);
	//LEftBottom
	leftNodeRow = row - 1;
	leftNodeCol = col - 1;
	AssignNeighbour(leftNodeRow, leftNodeCol, node, left_down);
	//RightBottom
	leftNodeRow = row + 1;
	leftNodeCol = col - 1;
	AssignNeighbour(leftNodeRow, leftNodeCol, node, right_down);
}


//Put neighbors to an ArralyList
bool A_star::AssignNeighbour(int x, int y, Node& parent, bool IScorner)
{
	if (x < 0 || y < 0 || x > g_terrain.GetWidth()-1 || y > g_terrain.GetWidth()-1)
		return false;
	if (g_terrain.IsWall(x, y))
		return true;
	if (IScorner)
		return true;
	
	//int index = GetFromOpenlist(x, y);
	m_grid[x][y].m_row = x;
	m_grid[x][y].m_col = y;
	Node tempnode;
	if (x == startnode.GetPositionX() && y == startnode.GetPositionY())
		return true;
	//assign parent for its neighbors
	if (m_grid[x][y].parent == NULL)
	{
		m_grid[x][y].parent = &m_grid[parent.GetPositionX()][parent.GetPositionY()];
		//tempnode = m_grid[x][y];
	}
	else
	{
		tempnode = m_grid[x][y];
		tempnode.parent = &m_grid[parent.m_row][parent.m_col];
		tempnode.SetHx(tempnode, goalnode, mtype);
		tempnode.SetGx();
		tempnode.SetFx(heuris);
	}

	
	
	//compute its cost
	m_grid[x][y].SetHx(m_grid[x][y], goalnode, mtype);
	m_grid[x][y].SetGx();
	m_grid[x][y].SetFx(heuris);
	if (m_grid[x][y].open == false)
	{
		if (m_grid[x][y].close == false)
		{
			//if child node is not on open or close list, then add to open list
			AddtoOpenlist(&m_grid[x][y]);
			if(!g_blackboard.GetOccupancyMapFlag())
				g_terrain.SetColor(x, y, DEBUG_COLOR_BLUE);
		}

		return false;
	}
	//if in open or close list
	else
	{
		//update old from new
		UpdateOpenlist(m_grid[x][y], tempnode);
	}
	
	return false;	
}

void A_star::UpdateOpenlist(Node &oldnode, Node &newnode)
{

	//update fx, if cheapest
	if (newnode.m_fx < oldnode.m_fx)
	{
		/*m_grid[oldnode.m_row][oldnode.m_col].m_fx = newnode.m_fx;
		m_grid[oldnode.m_row][oldnode.m_col].m_gx = newnode.m_gx;
		m_grid[oldnode.m_row][oldnode.m_col].parent = newnode.parent;*/
		m_grid[oldnode.m_row][oldnode.m_col] = newnode;
		//AddtoOpenlist(&m_grid[oldnode.m_row][oldnode.m_col]);
		m_openlist.push_back(m_grid[oldnode.m_row][oldnode.m_col]);
		for (size_t i = 0; i < m_openlist.size(); i++)
		{
			if (m_grid[oldnode.m_row][oldnode.m_col].m_row == m_openlist[i].m_row && m_grid[oldnode.m_row][oldnode.m_col].m_col == m_openlist[i].m_col)
			{
				m_openlist[i] = m_grid[oldnode.m_row][oldnode.m_col];
				break;
			}
		}
		m_openlist.pop_back();
	}

}

void A_star::AddtoOpenlist(Node *node)
{
	if (m_openlist.empty())
	{
		m_openlist.push_back(*node);
		m_openlist.back().open = true;
		node->open = true;
		return;
	}
	
	//it is not in openlist,then add into it
	m_openlist.push_back(*node);
	m_openlist.back().open = true;
	node->open = true;	
}
Node A_star::PopCheapNode(void)
{
	Node cheapnode;
	if (m_openlist.empty())
		return cheapnode;
	//O(n)
	cheapnode = m_openlist[0];
	if (m_openlist.size() <= 1)
	{
		m_openlist.pop_back();
		cheapnode.parent = NULL;
		return cheapnode;
	}
		
	int count = 0;
	for (size_t i = 0; i < m_openlist.size(); i++)
	{
		float fx = m_openlist[i].m_fx;
		if (cheapnode.m_fx > fx)
		{
			cheapnode = m_openlist[i];
			count  =  i;
			//break;
		}		
	}

	//replace with last, O(n)
	m_openlist[count] = m_openlist.back();
	//decrement last O(1)
	m_openlist.pop_back();
	return cheapnode;
}

bool A_star::Staright(Node &start, Node &goal)
{
	int xdiff = goal.m_row - start.m_row;
	int ydiff = goal.m_col - start.m_col;
	//go top
	if (xdiff == 0 && ydiff > 0)
	{
		for (int i = start.m_col; i <= goal.m_col; i++)
		{
			if (g_terrain.IsWall(start.m_row, i) == true)
				return false;
		}
			
		return true;
	}
	//go down
	if (xdiff == 0 && ydiff < 0)
	{
		for (int i = start.m_col; i >= goal.m_col; i--)
		{
			if (g_terrain.IsWall(start.m_row, i) == true)
				return false;
		}	
		return true;
	}
	//go topright
	if (ydiff >= 0 && xdiff > 0)
	{
		for (int i = start.m_row; i <= goal.m_row; i++)
			for (int j = start.m_col; j <= goal.m_col; j++)
				if (g_terrain.IsWall(i, j) == true)
					return false;
		return true;
	}
	//go topleft
	if (ydiff >= 0 && xdiff < 0)
	{
		for (int i = start.m_row; i >= goal.m_row; i--)
			for (int j = start.m_col; j <= goal.m_col; j++)
				if (g_terrain.IsWall(i, j) == true)
					return false;
		return true;
	}
	//go buttomright
	if (ydiff <= 0 && xdiff > 0)
	{
		for (int i = start.m_row; i <= goal.m_row; i++)
		{
			for (int j = start.m_col; j >= goal.m_col; j--)
			{
				if (g_terrain.IsWall(i, j) == true)
					return false;
			}
				
		}
		return true;
	}
	//go buttomleft
	if (ydiff < 0 && xdiff < 0)
	{
		for (int i = start.m_row; i >= goal.m_row; i--)
		{
			for (int j = start.m_col; j >= goal.m_col; j--)
			{
				if (g_terrain.IsWall(i, j) == true)
					return false;
			}
				
		}
			
		return true;
	}
	return false;
}


void A_star::Rubberband()
{
	//if the rubberpath only has 2 point, dont need rubber
	while (true)
	{
		//Make sure it deals with edge case such as ¡§less than 3 nodes in the path¡¨.
		if (m_rubberpath.size() <= 2)
			return;

		bool iterswitch = true;
		std::list<Node>::iterator iter1, iter2, start, middle , end;
		for (iter1 = m_rubberpath.begin(); iter1 != m_rubberpath.end(); iter1++)
		{
			iter2 = iter1;
			if (iterswitch == false && end == m_rubberpath.end())
				return;
			
			if (iterswitch)
			{
				//3 iters by order for dealing with 3 points
				start = iter2++;
				middle = iter2++;
				end = iter2;
				iterswitch = false;
			}

			if (Staright(*start, *end))
			{
				m_rubberpath.erase(middle);
				break;
			}
			else
			{
				start++; middle++;
				end++;
			}
		}
	}
}

void A_star::AddtoSmooth()
{
	while (!m_smoothlist.empty())
	{
		for (std::list<Point>::iterator iter = m_smoothlist.begin(); iter != m_smoothlist.end(); iter++)
		{
			std::list<Point>::iterator next = iter;
			next++;
			if (next == m_smoothlist.end())
				return;
			float distance = sqrt((next->x - iter->x) * (next->x - iter->x) + (next->y - iter->y) * (next->y - iter->y));
			//When adding points back. Distance between any two points should not be larger than 1.5 times grid width
			if (distance >= 1.5)
			{
				Point newpoint((next->x + iter->x) / 2, (next->y + iter->y) / 2);
				m_smoothlist.insert(next, newpoint);
				break;
			}
		}
	}	
}

void A_star::ComputeSmooth()
{
	//Make sure it deals with edge case such as ¡§less than 4 nodes in the path
	if (m_smoothlist.size() == 1)
	{
		std::list<Point>::iterator iter = m_smoothlist.begin();
		m_smoothpath.push_back(Point(iter->x, iter->y));
		return;
	}
	else if (m_smoothlist.size() == 2)
	{
		std::list<Point>::iterator iter = m_smoothlist.begin();
		std::list<Point>::iterator start = iter++, end = iter;
		
		//divide 2 point by 5
		//compute the vector for these 2 points
		D3DXVECTOR2 v1(start->x, start->y), v2(start->x, start->y), v3(end->x, end->y), v4(end->x, end->y);
		D3DXVECTOR2 result;
		//add first point into list
		m_smoothpath.push_back(Point(start->x, start->y));
		//compute 2nd point
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.25f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//compute 3rd
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.5f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//compute 4th
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.75f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//add last
		m_smoothpath.push_back(Point(end->x, end->y));
		return;
	}
	
	std::list<Point>::iterator iter = m_smoothlist.begin();
	std::list<Point>::iterator start = iter++, mid = iter++, end = iter;
	D3DXVECTOR2 v1(start->x, start->y), v2(start->x, start->y), v3(mid->x, mid->y), v4(end->x, end->y);
	D3DXVECTOR2 result;
	m_smoothpath.push_back(Point(start->x, start->y));
	//compute 2nd point
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.25f);
	m_smoothpath.push_back(Point(result.x, result.y));
	//compute 3rd
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.5f);
	m_smoothpath.push_back(Point(result.x, result.y));
	//compute 4th
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.75f);
	m_smoothpath.push_back(Point(result.x, result.y));

	if (m_smoothlist.size() == 3)
	{
		//compute the other 2 points
		D3DXVECTOR2 v1(start->x, start->y), v2(mid->x, mid->y), v3(end->x, end->y), v4(end->x, end->y);
		m_smoothpath.push_back(Point(start->x, start->y));
		//compute 2nd point
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.25f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//compute 3rd
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.5f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//compute 4th
		D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.75f);
		m_smoothpath.push_back(Point(result.x, result.y));
		//add last
		m_smoothpath.push_back(Point(end->x, end->y));
		return;
	}

		bool iterswitch = true;
		std::list<Point>::iterator newstart, newmid1, newmid2, newend;
		for (std::list<Point>::iterator newiter = m_smoothlist.begin(); newiter != m_smoothlist.end(); newiter++)
		{
			iter = newiter;
			if (iterswitch == false && newend == m_smoothlist.end())
				break;
			if (iterswitch)
			{
				newstart = iter++;
				newmid1 = iter++;
				newmid2 = iter++;
				newend = iter++;
				iterswitch = false;
			}
			v1.x = newstart->x, v1.y = newstart->y, v2.x = newmid1->x, v2.y = newmid1->y;
			v3.x = newmid2->x, v3.y = newmid2->y, v4.x = newend->x, v4.y = newend->y;
			
			m_smoothpath.push_back(Point(newmid1->x, newmid1->y));
			//compute 2nd point
			D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.25f);
			m_smoothpath.push_back(Point(result.x, result.y));
			//compute 3rd
			D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.5f);
			m_smoothpath.push_back(Point(result.x, result.y));
			//compute 4th
			D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.75f);
			m_smoothpath.push_back(Point(result.x, result.y));
			//go next iterator
			newstart++; newmid1++; newmid2++; newend++;
		}

	//deal with last 4 points
	v1.x = newstart->x, v1.y = newstart->y, v2.x = newmid1->x, v2.y = newmid1->y, v3.x = newmid2->x, v3.y = newmid2->y, v4.x = newmid2->x, v4.y = newmid2->y;
	m_smoothpath.push_back(Point(newmid1->x, newmid1->y));
	//compute 2nd point
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.25f);
	m_smoothpath.push_back(Point(result.x, result.y));
	//compute 3rd
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.5f);
	m_smoothpath.push_back(Point(result.x, result.y));
	//compute 4th
	D3DXVec2CatmullRom(&result, &v1, &v2, &v3, &v4, 0.75f);
	m_smoothpath.push_back(Point(result.x, result.y));
	//add last
	m_smoothpath.push_back(Point(newmid2->x, newmid2->y));
}

D3DXVECTOR3 NewGetCoordinates(float r, float c, int m_width)
{
	D3DXVECTOR3 pos;

	const float offset = 1.f / m_width / 2.f;

	pos.x = ((float)c / (float)m_width) + offset;
	pos.y = 0.0f;
	pos.z = ((float)r / (float)m_width) + offset;

	return(pos);
}

bool Movement::ComputePath( int r, int c, bool newRequest )
{
	m_goal = g_terrain.GetCoordinates( r, c );
	m_movementMode = MOVEMENT_WAYPOINT_LIST;
	
	bool useAStar = true;
	//bool useAStar = false;
	if( useAStar )
	{
		///////////////////////////////////////////////////////////////////////////////////////////////////
		//INSERT YOUR A* CODE HERE
		//1. You should probably make your own A* class.
		//2. You will need to make this function remember the current progress if it preemptively exits.
		//3. Return "true" if the path is complete, otherwise "false".
		///////////////////////////////////////////////////////////////////////////////////////////////////
		
		//set startnod and goalnode info
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn(&cur, &curR, &curC);
		startnode(curR, curC); 
		goalnode(r, c);
		heuris = GetHeuristicWeight();
		type = GetHeuristicCalc();
		mtype = static_cast<Heuristic_cal> (type);
		astar_analysis = GetAnalysis();
		//NEWREQUEST
		if (newRequest)
		{
			//reset all path
			a_star.ClearAllpath();
			m_waypointList.clear();
			a_star.m_grid[startnode.m_row][startnode.m_col].init();
			a_star.m_grid[startnode.m_row][startnode.m_col] = startnode;
			a_star.m_grid[startnode.m_row][startnode.m_col].parent = NULL;
			a_star.m_grid[startnode.m_row][startnode.m_col].m_gx = 0.0f;
			a_star.m_grid[startnode.m_row][startnode.m_col].SetHx(startnode, goalnode, mtype);
			a_star.m_grid[startnode.m_row][startnode.m_col].SetFx(heuris);
			//Push Start Node onto the Open List
			a_star.AddtoOpenlist(&a_star.m_grid[startnode.m_row][startnode.m_col]);
		}

		if (g_terrain.IsWall(goalnode.GetPositionX(), goalnode.GetPositionY()) == true || 
			g_terrain.IsWall(startnode.GetPositionX(), startnode.GetPositionY()) == true)
			return false;
		//straight line path
		if (m_straightline)
		{
			if (a_star.Staright(startnode, goalnode))
			{
				D3DXVECTOR3 spot = g_terrain.GetCoordinates(goalnode.GetPositionX(), goalnode.GetPositionY());
				m_waypointList.push_back(spot);
				return true;
			}
		}
		bool isFind = a_star.Findpath(startnode, goalnode);
		if (isFind)
		{
			if (m_rubberband)
			{
				//draw rubberbound
				//reverse copy
				for (std::list<Node>::reverse_iterator Iterator = a_star.m_A_path.rbegin(); Iterator != a_star.m_A_path.rend(); ++Iterator)
				{
					Node temp(Iterator->m_row, Iterator->m_col);
					a_star.m_rubberpath.push_back(temp);
				}
				a_star.Rubberband();
				a_star.m_A_path.clear();
				//reverse copy back to a star path
				for (std::list<Node>::reverse_iterator Iterator = a_star.m_rubberpath.rbegin(); Iterator != a_star.m_rubberpath.rend(); ++Iterator)
				{
					Node temp(Iterator->m_row, Iterator->m_col);
					a_star.m_A_path.push_back(temp);
				}
			}

			if (m_smooth)
			{
				//draw smooth
				for (std::list<Node>::iterator iter = a_star.m_A_path.begin(); iter != a_star.m_A_path.end(); ++iter)
					a_star.m_smoothlist.push_back(Point((float)iter->m_row, (float)iter->m_col));
				//Add points back in 
				a_star.AddtoSmooth();
				a_star.ComputeSmooth();
				for (std::list<Point>::iterator iter = a_star.m_smoothpath.begin(); iter != a_star.m_smoothpath.end(); ++iter)
				{
					D3DXVECTOR3 spot = NewGetCoordinates(iter->x, iter->y, g_terrain.GetWidth());
					m_waypointList.push_back(spot);
				}
			}
			else
			{
				//draw path no smooth normally
				for (std::list<Node>::iterator Iterator = a_star.m_A_path.begin(); Iterator != a_star.m_A_path.end(); ++Iterator)
				{
					D3DXVECTOR3 spot = g_terrain.GetCoordinates(Iterator->m_row, Iterator->m_col);
					m_waypointList.push_back(spot);
				}
			}
			return true;
		}
		
		//cant find path so end seraching time
		if (a_star.foundpath == false)
		{
			D3DXVECTOR3 spot = g_terrain.GetCoordinates(startnode.GetPositionX(), startnode.GetPositionY());
			m_waypointList.push_back(spot);
			return true;
		}
		
		return false;
		
	}
	else
	{	
		//Randomly meander toward goal (might get stuck at wall)
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn( &cur, &curR, &curC );

		m_waypointList.clear();
		m_waypointList.push_back( cur );

		int countdown = 100;
		while( curR != r || curC != c )
		{
			if( countdown-- < 0 ) { break; }

			if( curC == c || (curR != r && rand()%2 == 0) )
			{	//Go in row direction
				int last = curR;
				if( curR < r ) { curR++; }
				else { curR--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curR = last;
					continue;
				}
			}
			else
			{	//Go in column direction
				int last = curC;
				if( curC < c ) { curC++; }
				else { curC--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curC = last;
					continue;
				}
			}

			D3DXVECTOR3 spot = g_terrain.GetCoordinates( curR, curC );
			m_waypointList.push_back( spot );
			g_terrain.SetColor( curR, curC, DEBUG_COLOR_YELLOW );
		}
		return true;
	}
}
