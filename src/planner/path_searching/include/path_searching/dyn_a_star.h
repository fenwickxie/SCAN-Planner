#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

/** A* 的返回状态；调用方据此区分输入无效与搜索失败。 */
enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	// 节点池会被重复使用。rounds 记录节点属于哪一轮搜索，避免每次搜索前
	// 清空整个 100^3 节点池，这是在线重规划中很重要的常数级优化。
	int rounds{0};
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	// gScore 是起点到当前节点的累计代价，fScore = gScore + heuristic。
	double gScore{inf}, fScore{inf};
	// 用父指针在找到终点后逆向恢复离散路径。
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	// 与连续优化器共享同一张膨胀占用地图，搜索结果天然包含机器人外形裕量。
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	// 搜索坐标系以碰撞区段中点为原点。step_size 通常等于地图分辨率，
	// inv_step_size 缓存倒数以减少高频坐标转换中的除法。
	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	// 略大于 1 的 tie breaker 偏向离终点更近的同代价节点，减少扩展数量。
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	// 固定节点池避免每轮重规划进行大量动态分配；有效搜索窗口由 POOL_SIZE 决定。
	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	/** 绑定占用地图并一次性分配搜索节点池。 */
	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	/**
	 * 在碰撞区段起终点之间搜索绕障路径。
	 * 当前实现扩展 XY 八邻域，Z 由起终点间的线性高度平面确定，
	 * 因而是“带坡度约束的 2.5D 搜索”，不是自由 26 邻域三维搜索。
	 */
	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	// 对角距离与 8 邻域移动代价一致；tie breaker 仅用于打破等价节点。
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	// 节点池中心对应当前碰撞区段中点，因此同一节点池可服务任意世界位置。
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	// 加 0.5 后转整数实现最近邻量化；超出固定池代表本轮搜索窗口不够大。
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
