#include "Astar.h"

namespace pathplanning{

void Astar::InitAstar(Mat& _Map, AstarConfig _config)
{
    Mat Mask;
    InitAstar(_Map, Mask, _config);
}

void Astar::InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config)
{
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            { 0, -1},          { 0, 1},
            { 1, -1}, { 1, 0}, { 1, 1}
    };

    Map = _Map;
    config = _config;
    neighbor = Mat(8, 2, CV_8S, neighbor8).clone();

    MapProcess(Mask);
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path, vector<double>& orientations)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();

    // Get the path and orientations
    if (TailNode) {
        GetPath(TailNode, path, orientations);
    }
}

void Astar::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }

    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

Node* Astar::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);  
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(config.Euclidean)
                {
                    int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                    H = round(10 * sqrt(dist2));
                }
                else
                {
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::GetPath(Node* TailNode, vector<Point>& path, vector<double>& orientations)
{
    path.clear();
    orientations.clear(); // clear vector orientations

    // Traverse from TailNode (target) to start and store the path in reverse order
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        path.push_back(CurNode->point);
        CurNode = CurNode->parent;
    }

    // Reverse the path to go from start to target
    std::reverse(path.begin(), path.end());

    // Handle edge case for single-point path
    if (path.size() == 1) 
    {
        orientations.push_back(0.0); // Default orientation for a single point (no movement)
        return;
    }

    // Calculate orientation for the first point (start)
    {
        Point nextPoint = path[1];
        Point startPoint = path[0];
        int dx = nextPoint.x - startPoint.x;
        int dy = nextPoint.y - startPoint.y;
        double startOrientation = atan2(dy, dx);
        orientations.push_back(startOrientation);
    }

    // Calculate orientations for the path from the second point to the last point
    for(size_t i = 1; i < path.size() - 1; ++i)
    {
        Point prevPoint = path[i-1];
        Point curPoint = path[i];
        Point nextPoint = path[i+1];
        int dx = nextPoint.x - curPoint.x;
        int dy = nextPoint.y - curPoint.y;
        double orientation = atan2(dy, dx);
        orientations.push_back(orientation);
    }

    // Calculate orientation for the last point (end)
    {
        Point prevPoint = path[path.size() - 2];
        Point endPoint = path.back();
        int dx = endPoint.x - prevPoint.x;
        int dy = endPoint.y - prevPoint.y;
        double endOrientation = atan2(dy, dx);
        orientations.push_back(endOrientation);
    }

    // Release memory
    while(!OpenList.empty()) {
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}


}