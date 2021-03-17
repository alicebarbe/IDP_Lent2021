#include "PathFindNode.h"

PathFindNode::PathFindNode() :
	m_parent(nullptr)
{}

PathFindNode::~PathFindNode()
{}

PathFindNode* PathFindNode::getParent() const
{
	return m_parent;
}

std::vector<std::pair<PathFindNode*, float>>& PathFindNode::getChildren()
{
	return m_children;
}

void PathFindNode::addChild(PathFindNode* child, float distance)
{
	m_children.push_back(std::make_pair(child,distance));
}

void PathFindNode::clearChildren()
{
	m_children.clear();
}

void PathFindNode::setParent(PathFindNode* parent)
{
	m_parent = parent;
}
