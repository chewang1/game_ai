/* Copyright Steve Rabin, 2008. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2008"
 */

#pragma once

class GameObject;
class Node;
class A_star;
typedef std::list<D3DXVECTOR3> WaypointList;

enum MovementMode
{
	MOVEMENT_NULL,
	MOVEMENT_SEEK_TARGET,
	MOVEMENT_WAYPOINT_LIST
};


class Movement
{
	friend class PathfindingTests;

public:
	Movement( GameObject& owner );
	~Movement( void );

	inline void SetTarget( D3DXVECTOR3& target )			{ m_target = target; }
	inline D3DXVECTOR3& GetTarget( void )					{ return( m_target ); }

	void Animate( double dTimeDelta );
	void DrawDebugVisualization( IDirect3DDevice9* pd3dDevice, D3DXMATRIX* pViewProj );

	void SetIdleSpeed( void );
	void SetWalkSpeed( void );
	void SetJogSpeed( void );

	bool ComputePathWithTiming( int r, int c, bool newRequest );
	void SetHeuristicWeight( float value )					{ m_heuristicWeight = value; }
	float GetHeuristicWeight() const                        { return m_heuristicWeight; }
	void SetHeuristicCalc( int value )						{ m_heuristicCalc = value; }
	int GetHeuristicCalc() const                            { return m_heuristicCalc; }

	void SetSmoothPath( bool enable )						{ m_smooth = enable; }
	bool GetSmoothPath() const                              { return m_smooth; }
	void SetRubberbandPath( bool enable )					{ m_rubberband = enable; }
	bool GetRubberbandPath() const                          { return m_rubberband; }
	void SetStraightlinePath( bool enable )					{ m_straightline = enable; }
	bool GetStraightlinePath() const                        { return m_straightline; }
	void SetSingleStep( bool enable )					    { m_singleStep = enable; }
	bool GetSingleStep() const                              { return m_singleStep; }
	void SetExtraCredit( bool enable )						    { m_extracredit = enable; }
	bool GetExtraCredit()									    { return m_extracredit; }
	void AStarUsesAnalysis( bool enable )					{ m_aStarUsesAnalysis = enable; }
	bool GetAnalysis() const                                { return m_aStarUsesAnalysis; }
	void SetFogOfWar( bool enable )                         { m_fogOfWar = enable; }
	bool GetFogOfWar() const                                { return m_fogOfWar; }

protected:

	GameObject* m_owner;

	bool m_smooth;
	bool m_rubberband;
	bool m_straightline;
	bool m_singleStep;
	bool m_extracredit;
	bool m_aStarUsesAnalysis;
	bool m_fogOfWar;
	float m_heuristicWeight;
	int m_heuristicCalc;
	MovementMode m_movementMode;

	D3DXVECTOR3 m_target;
	D3DXVECTOR3 m_goal;

	WaypointList m_waypointList;

	float m_speedWalk;
	float m_speedJog;

	bool ComputePath( int r, int c, bool newRequest );
};

enum Heuristic_cal
{
	Euclidean = 0,
	Octile = 1,
	Chebyshev = 2,
	Manhattan = 3,
};
class Node
{
public:
	friend class Movement;
	Node();
	Node(int r, int c);

	void operator=(Node &rhs);
	void operator()(int &r, int &c);
	void SetGx();
	void SetHx(Node &curnode, Node &goalnode, Heuristic_cal type);
	void SetFx(float weight);

	int GetPositionX();
	int GetPositionY();
	void SetPositionXY(Node &rhs);
	void init();
	float GetFx() { return m_fx; };

	int m_row, m_col;
	float m_fx;
	float m_gx;
	float m_hx;
	Node *parent;
	bool open, close;
};

struct Point
{
	float x, y;
	Point() { x = y = 0.0f; }
	Point(float midx, float midy) : x(midx), y(midy){};
};
class A_star
{
public:

	friend class Node;

	A_star();
	std::vector<Node> m_openlist;
	std::list<Node> m_A_path;

	Node m_grid[40][40];
	bool foundpath;
	bool Findpath(Node &start, Node &goal);
	void StoreInApath(Node node);
	Node PopCheapNode(void);
	void AddtoOpenlist(Node *node);
	void GetNeighbours(Node &node);
	bool AssignNeighbour(int x, int y, Node& node, bool IScorner);

	void UpdateOpenlist(Node &oldnode, Node &newnode);
	void ClearAllpath();
	//starightline
	bool Staright(Node &start, Node &goal);
	//rubberband
	void Rubberband();
	
	void AddtoSmooth();
	void ComputeSmooth();
    
    std::list<Node> m_rubberpath;

	//for smooth usage
	//when we get all points we use nre list to store and convert to a_ptah
	std::list<Point> m_smoothpath;
	//its the list for smooth to add points
	std::list<Point> m_smoothlist;
};