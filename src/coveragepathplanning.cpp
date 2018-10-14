//
//  coveragepathplanning.cpp
//  CPP
//
//  Created by 欧阳伟斌 on 16/10/17.
//  Copyright © 2016年 欧阳伟斌. All rights reserved.
//

#include "coveragepathplanning.hpp"
#include <queue>
#include <stack>

using namespace cv;   //  NOLINT
using namespace std;  //  NOLINT
using namespace cpp;  //  NOLINT

static const int CLOSED = -1;
static const unsigned char BLOCKED = 0;

static const Point g_neighbours[8] = {Point(1, 0),  Point(1, 1),  Point(0, 1),
                                      Point(-1, 1), Point(-1, 0), Point(-1, -1),
                                      Point(0, -1), Point(1, -1)};

static int WaveFront(
    const Mat &_binary, Mat &_dt, const Point &_tar) {  //  NOLINT
  // init
  _dt =
      Mat(_binary.rows, _binary.cols, CV_32S,
          CLOSED);  // prepare matrix with same size to binary image
  Mat binary = _binary.clone();  // do not touch original source
  queue<Point> closed;
  Point cur;
  const Point *neighbours = g_neighbours;
  int dist;
  int col, row;

  // set initial distance
  _dt.at<int>(_tar.y, _tar.x) = 0;
  binary.at<unsigned char>(_tar.y, _tar.x) = 0;
  closed.push(_tar);

  // update neighbour distance
  while (!closed.empty()) {
    // get current point and its distance
    cur = closed.front();
    dist = _dt.at<int>(cur.y, cur.x);

// traverse current point's neighbour
#ifdef WAVEFRONT_CONNECT_8
    for (int i = 0; i < 8; ++i) {
#else
    for (int i = 0; i < 8; i += 2) {
#endif
      col = cur.x + neighbours[i].x;
      row = cur.y + neighbours[i].y;

      // check if valid and unvisited
      if (0 <= row && row < binary.rows && 0 <= col && col < binary.cols &&
          BLOCKED != binary.at<unsigned char>(row, col)) {
        _dt.at<int>(row, col) = dist + 1;              // update distance
        binary.at<unsigned char>(row, col) = BLOCKED;  // set visited
        closed.push(Point(col, row));
      }
    }

    closed.pop();
  }

  return 0;
}

static int IsValid(const Mat &_dt, int row, int col, int _radius) {
  // init
  int dist = 2 * _radius;
  int anchorX = col - _radius;
  int anchorY = row + _radius;
  int x, y;

  // traverse neighbours
  for (int i = 0; i <= dist; ++i) {
    x = anchorX + i;
    for (int j = 0; j >= -dist; --j) {
      y = anchorY + j;
      if (y < 0 || x < 0 || y >= _dt.rows || x >= _dt.cols ||
          CLOSED == _dt.at<int>(y, x)) {
        return 0;
      }
    }
  }

  return 1;
}

static int FindHighestNeighbour(
    const Mat &_dt,                                                  //  NOLINT
    const Point &_center, Point &_tar, int _radius, int &_direct) {  //  NOLINT
  // init
  int i, j, col, row, tarY = -1, tarX = -1, maxDist = -99, maxDistIndex = -1;
  const Point *neighbours = g_neighbours;
  static int start = 0;

// traverse center's neighbour and find the highest one
#ifdef PLANNING_CONNECT_8
  for (j = 0; j < 8; ++j) {
    i = (start + j) % 8;
#else
  for (j = 0; j < 4; ++j) {
    i = (start + 2 * j) % 8;
#endif

    // calculate candidate's coordinate
    col = _center.x + neighbours[i].x * (2 * _radius + 1);
    row = _center.y + neighbours[i].y * (2 * _radius + 1);

    if (col < 0 || row < 0 || col >= _dt.cols ||
        row >= _dt.rows) {  // ignore invalid point
      continue;
    }

    if (_dt.at<int>(row, col) > maxDist &&
        IsValid(_dt, row, col, _radius)) {  // found new max choice
      // update max distance information
      maxDist = _dt.at<int>(row, col);
      maxDistIndex = i;
      tarX = col;
      tarY = row;

#ifndef PLANNING_SOLID_BEGINNING
#ifndef PLANNING_ONE_ACCUMULATE_BEGINNING
      start = (i + 4) % 8;
// start = (i + 2) % 8;
#endif
#endif
    }
  }

  // update supposed direction
  _direct = maxDistIndex;

  if (-1 == maxDistIndex) {  // not found
    return -1;
  }

  // update target point
  _tar.x = tarX;
  _tar.y = tarY;

#ifndef PLANNING_SOLID_BEGINNING
#ifdef PLANNING_ONE_ACCUMULATE_BEGINNING
  start = (maxDistIndex + 4) % 8;
// start = (maxDistIndex + 2) % 8;
#endif
#endif

  return 0;
}

static int Visit(
    Mat &_dt, stack<Point> &_track,          //  NOLINT
    deque<Point> &_path, const Point &_tar,  //  NOLINT
    int _radius, bool isSameDirect) {        //  NOLINT
  // init
  int dist = 2 * _radius;
  int anchorX = _tar.x - _radius;
  int anchorY = _tar.y + _radius;
  int x, y;

  // close points in radius
  for (int i = 0; i <= dist; ++i) {
    for (int j = 0; j >= -dist; --j) {
      x = anchorX + i;
      y = anchorY + j;
      if (y < 0 || x < 0 || y >= _dt.rows ||
          x >= _dt.cols) {  // ignore invalid point
        continue;
      }
      _dt.at<int>(y, x) = CLOSED;
    }
  }

  // update track
  _track.push(_tar);

  // update path
  if (isSameDirect) {
    _path.pop_back();
  }
  _path.push_back(_tar);

  return 0;
}

static int Planning(
    const Mat &_dt, const Point &_start, const Point &_goal,
    deque<Point> &_path, int _radius) {  //  NOLINT
  // init
  stack<Point> track;
  Mat dt = _dt.clone();  // do not touch original source
  int rst;
  Point tar;
  bool isSameDirect = false;
  int direct = -1, curDirect = -1, lastDirect = -1;

  // close goal
  dt.at<int>(_goal.y, _goal.x) = CLOSED;

  // visit start point
  rst = Visit(dt, track, _path, _start, _radius, 0);

  // path planning
  while (!track.empty()) {
    rst = FindHighestNeighbour(dt, track.top(), tar, _radius, direct);

    // update direction information
    isSameDirect = (curDirect == direct && -1 != lastDirect);
    lastDirect = curDirect;
    curDirect = direct;

    if (0 == rst) {  // found
      Visit(dt, track, _path, tar, _radius, isSameDirect);
    } else {  // not found
      track.pop();
    }
  }

  // add goal to path
  _path.push_back(_goal);

  return 0;
}

int cpp::CoveragePathPlanning(
    const Mat &_binary, const Point &_start, const Point &_goal,
    deque<Point> &_path, int _radius) {  //  NOLINT
  Mat binary = _binary.clone();
  int rst = -1;

  Mat dt;  // distance transform
  if (0 != (rst = WaveFront(binary, dt, _goal))) {
    return -1;
  }

  if (0 != (rst = Planning(dt, _start, _goal, _path, _radius))) {
    return -1;
  }

  return 0;
}

/* ---------------------------------------------------------- */

struct Candidate
{
    Point m_pos;
    bool m_checkUp;
    bool m_checkDown;
};

// robot could be in here?
static int IsValid(const Mat &_dt, Point _tar, int _radius) {
    //init
	// robot box
    int dist = 2 * _radius + 1;
    int anchorX = _tar.x - _radius;
    int anchorY = _tar.y + _radius;
    int x, y;
    
    //traverse neighbours
    for (int i = 0; i <= dist; ++i) {
        x = anchorX + i;
        for (int j = 0; j >= -dist; --j) {
            y = anchorY + j;
			// box should be in map
            if (y < 0 || x < 0 || y >= _dt.rows || x >= _dt.cols || 
              0 == _dt.at<unsigned char>(y, x)) {
                return 0;
            }
        }
    }
    
    return 1;
}

int UpdatePath(cv::Mat &_binary, const cv::Point &_tar, 
  std::deque<cv::Point> &_path, stack<Candidate> &_trace, int _radius) {
    // search left
    Point left = _tar;
    int left_dist = -1;
	// find left end location that robot could go
    while (IsValid(_binary, left, _radius)) {
        --left.x;
        ++left_dist;
    }
    ++left.x;

    // search right
    Point right = _tar;
    int right_dist = -1;
    while (IsValid(_binary, right, _radius)) {
        ++right.x;
        ++right_dist;
    }
    --right.x;

    // update
    Candidate tmp;
    if (left_dist < right_dist) { // from left to right
        _path.push_back(left);
        _path.push_back(right);
        for (int i = left.x; i <= right.x; ++i) {
            _binary.at<unsigned char>(_tar.y, i) = 0;
            tmp.m_pos.x = i;
            tmp.m_pos.y = _tar.y;
            tmp.m_checkUp = false;
            tmp.m_checkDown = false;
            _trace.push(tmp);
        }
    } else { // from right to left
        _path.push_back(right);
        _path.push_back(left);
        for (int i = right.x; i >= left.x; --i) {
            _binary.at<unsigned char>(_tar.y, i) = 0;
            tmp.m_pos.x = i;
            tmp.m_pos.y = _tar.y;
            tmp.m_checkUp = false;
            tmp.m_checkDown = false;
            _trace.push(tmp);
        }
    }

    return 0;
}

void insert_a_star_path(const cv::Mat &_binary, std::deque<cv::Point> &_path, int _radius, bool _for_goal = false, const cv::Point &_goal = Point(-1, -1)) {
	Mat image = _binary.clone();
	//flip(image, image, 1);
	//rows and cols store the width and height of the image respectively
	int rows , cols;
	
	rows = image.rows;
	cols = image.cols;
	
	// FIXME: too big size image -> process died!
	//G[i][j] denotes the category to which the pixel (i,j) belong to
	//this will be used map?(start point, end point, free, non-free)
	//int G[rows][cols];
	char **G = new char*[rows];
	for(int i = 0; i < rows; i++) {
		G[i] = new char[cols];
	}

	
	//Parent[i][j] denotes the parent of the pixel (i,j)
	//pair<int,int> Parent[rows][cols];
	pair<int, int> **Parent = new pair<int, int>*[rows];
	for(int i = 0; i < rows; i++) {
		Parent[i] = new pair<int, int>[cols];
	}
	
	//Closed_List[i][j] denotes whether the pixel (i,j) belongs to the closed list(should not be looked at again)
	//and whose final distance is calculated
	//int Closed_List[rows][cols];
	int **Closed_List = new int*[rows];
	for(int i = 0; i < rows; i++) {
		Closed_List[i] = new int[cols];
	}
	

	int path_size = _path.size();
	pair<int,int> start,destination;
	// find start point and destination point of a star path
	// first -> y, second -> x
	if(!_for_goal) {
		start = make_pair(_path.at(path_size - 3).y, _path.at(path_size - 3).x);
		destination = make_pair(_path.at(path_size - 2).y, _path.at(path_size - 2).x);
	} else {
		start = make_pair(_path.at(path_size - 1).y, _path.at(path_size - 1).x);
		destination = make_pair(_goal.y, _goal.x);
	}
	
	
	// cost
	struct dist {
		//g is the movement cost to move from starting point to that point
		int g;
		//h is the diagonal shortcut distance (estimated movement cost to move from that point to the destination)
		//h is just a guess and ignores all the obstacles and other conditions
		int h;
		//f is the addition of g and h
		int f;
	};
	
	//Distance[i][j] will store the 3 different distance of the pixel(i,j)
	//dist Distance[rows][cols];
	dist **Distance= new dist*[rows];
	for(int i = 0; i < rows; i++) {
		Distance[i] = new dist[cols];
	}
	
	//Initialisation
	for(int i=0 ; i<rows ; i++) {
		for(int j=0 ; j<cols ; j++) {
			G[i][j] = 1; //white pixel
			Distance[i][j].f = INT_MAX; //initialise each point to be at max distance from the start point
			Closed_List[i][j] = 0; //no pixel is on closed list yet
		}
	}
	
	//Temporary variables to store the bgr properties of a pixel
	int bvalue,gvalue,rvalue;
	
	//Categorise the image into different regions
	for(int i=0 ; i<rows ; i++) {
		for(int j=0 ; j<cols ; j++) {
			if(image.at<unsigned char>(i, j) == 0) {
				G[i][j] = 0;
			}
			//FIXME
			if(i == start.first && j == start.second) {
				G[i][j] = 2; // start point
				circle(image, Point(j, i), 3, Scalar(0, 255, 0), -1);
			}
			else if(i == destination.first && j == destination.second) {
				G[i][j] = 3; // goal point
				circle(image, Point(j, i), 3, Scalar(0, 0, 255), -1);
			}
			/*
			bvalue = image.at<Vec3b>(i,j)[0];
			gvalue = image.at<Vec3b>(i,j)[1];
			rvalue = image.at<Vec3b>(i,j)[2];
			if((bvalue<50)&&(gvalue>200)&&(rvalue<50)) {
				G[i][j] = 2; //green pixel
			}
			else if((bvalue<50)&&(gvalue<50)&&(rvalue>200)) {
				G[i][j] = 3; //red pixel
			}
			else if((bvalue<50)&&(gvalue<50)&&(rvalue<50)) {
				G[i][j] = 0; //black pixel
			}
			*/
		}
	}
	
	/*
	int start_count = 0;	//Stores the number of starting points
	int start_x_sum = 0;	//Stores the sum of x-coordinates of all the starting points
	int start_y_sum = 0;	//Stores the sum of y-coordinates of all the starting points
	int destination_count = 0;	//Stores the number of destination points
	int destination_x_sum = 0;	//Stores the sum of x-coordinates of all the destination points
	int destination_y_sum = 0;	//Stores the sum of y-coordinates of all the destination points
	
	//start and destination denotes the starting and destination points respectively
	
	//find the start and destination points using the idea of centre of mass where each point is of unit mass
	for(int i=0 ; i<rows ; i++) {
		for(int j=0 ; j<cols ; j++) {
			if(G[i][j]==2) {	//start(green) pixel
				start_x_sum = start_x_sum + i;
				start_y_sum = start_y_sum + j;
				start_count++;
			}
			else if(G[i][j]==3) {	//destination(red) pixel
				destination_x_sum = destination_x_sum + i;
				destination_y_sum = destination_y_sum + j;
				destination_count++;
			}
		}
	}
	
	int start_x = start_x_sum/start_count;				//x-coordinate of starting point
	int start_y = start_y_sum/start_count;				//y-coordinate of starting point
	int destination_x = destination_x_sum/destination_count;	//x-coordinate of destination point
	int destination_y = destination_y_sum/destination_count;	//y-coordinate of destination point
	*/

	//start = make_pair(_path.at(path_size - 3).y, _path.at(path_size - 3).x);
	//destination = make_pair(_path.at(path_size - 2).y, _path.at(path_size - 2).x);
	
	
	// TODO: x <-> y renaming
	//Temporary variables to store the distance in X and Y directions
	int xDist , yDist;
	
	//Calculate h distances by diagonal shortcut method
	for(int i=0 ; i<rows ; i++) {
		for(int j=0 ; j<cols ; j++) {
			//diagonal shortcut distance
			xDist = abs(i-start.first);
			yDist = abs(j-start.second);
			if(xDist>yDist) {
				Distance[i][j].h = 14*yDist + 10*(xDist-yDist);
			}
			else {
				Distance[i][j].h = 14*xDist + 10*(yDist-xDist);
			}
		}
	}
	
	//To store the value of distances in sorted order
	priority_queue<pair<int,pair<int,int> > , vector<pair<int,pair<int,int> > > , greater<pair<int,pair<int,int> > > > Q;
	
	//Temporary pair to extract the topmost pair of the queue
	pair<int,pair<int,int> > top;
	
	//////////////////////////////////////
	// TODO: x <-> y , xnext <-> ynext renaming
	
	//Temporary variables whose names describe the value stored by them
	int x , y , xnext , ynext , dist_next_g , dist_next_f , d ;
	
	//(N,M,w)==>(i,j,weight) to traverse the 8 adjacent vertices of a point
	int N[] = {-1,0,1,-1,1,-1,0,1};
	int M[] = {-1,-1,-1,0,0,1,1,1};
	int W[] = {14,10,14,10,10,14,10,14};
	
	//f and g distance of start point is zero
	Distance[start.first][start.second].g = 0;
	Distance[start.first][start.second].f = 0;
	
	//Parent of starting point is starting point itself
	Parent[start.first][start.second] = make_pair(start.first,start.second);
	
	//Initiate exploration from starting point
	Q.push(make_pair(Distance[start.first][start.second].f , make_pair(start.first,start.second)));
	
	//Carry-on exploration
	while((!Q.empty()) && (!Closed_List[destination.first][destination.second]) ) {
		top = Q.top();
		d = top.first;
		x = top.second.first;
		y = top.second.second;
		Q.pop();
		if(!Closed_List[x][y]) {
			Closed_List[x][y] = 1;
			for(int k=0 ; k<8 ; k++) {
				xnext = x + N[k];
				ynext = y + M[k];
				if((xnext>=0)&&(xnext<rows)&&(ynext>=0)&&(ynext<cols)) {
					if((G[xnext][ynext])&&(!Closed_List[xnext][ynext])) {
						dist_next_g = W[k]+Distance[x][y].g;
						dist_next_f = dist_next_g + Distance[xnext][ynext].h;
						if(dist_next_f<Distance[xnext][ynext].f) {
							Distance[xnext][ynext].g = dist_next_g;
							Distance[xnext][ynext].f = dist_next_f;
							Parent[xnext][ynext] = make_pair(x,y);
							Q.push(make_pair(dist_next_f , make_pair(xnext,ynext)));
						}
					}
				}
			}
			
		}
	}
	
	//Initialise a temporary pair to traverse through the path found
	pair<int,int> temp = destination;

	// element sequence: goal point -> .... -> start point
	vector<cv::Point> all_waypoint;
	
	//Traverse through the path and mark it on the image
	while(temp!=Parent[temp.first][temp.second]) {
		/*
		x = temp.first;		// real meaning: y
		y = temp.second;
		image.at<Vec3b>(x,y)[0] = 0;
		image.at<Vec3b>(x,y)[1] = 250;
		image.at<Vec3b>(x,y)[2] = 0;
		*/

		all_waypoint.push_back(Point(temp.second, temp.first));
		temp = Parent[temp.first][temp.second];
	}

	///////////////////////////////////

	// TODO: only use node in graph
	int waypoint_count = all_waypoint.size();
	deque<cv::Point>::iterator it;
	if(!_for_goal) {
		it = _path.end() - 2;
	}else {
		it = _path.end() - 1;
	}

	for(int i = 0; i < waypoint_count; i++) {
		// insert all waypoints sequentially
		it = _path.insert(it, all_waypoint[waypoint_count - i - 1]);
		it++;
	}


	//Display the final image after highlighting the path found
	//display_image(image,"Output");
	//imshow("Output",image); 
	//waitKey(0);

	// memory deallocation
	for(int i = 0; i < rows; i++) {
		delete [] G[i];
		delete [] Parent[i];
		delete [] Closed_List[i];
		delete [] Distance[i];
	}
	delete [] G;
	delete [] Parent;
	delete [] Closed_List;
	delete [] Distance;
}


// @param _tar start point
// @param _radius number of pixel for robot radius
int cpp::ZigZagPathPlanning(const cv::Mat &_binary, 
  const cv::Point &_tar, const cv::Point &_goal, std::deque<cv::Point> &_path, int _radius) {
	// binary -> searched area pixel value will be 0
    Mat binary = _binary.clone();
    stack<Candidate> trace;

	// robot start point validation check
    if (IsValid(binary, _tar, _radius) != 1) {
        return -1;
    }

    if (0 != UpdatePath(binary, _tar, _path, trace, _radius)) {
        return -1;
    }

	bool search_dir_change = false;
	bool inst_before_up = false;
	bool inst_before_down = false;

	// trace: all of the pixel which is on path
	// step: loop[search up -> (invalid) -> search down -> (invaild)] -> return
    while (trace.empty() != true) {
        //  search up
        if (trace.top().m_checkUp != true) {
            trace.top().m_checkUp = true;
			// trace.top() - one line trace path end point
            Point tmp(trace.top().m_pos.x, trace.top().m_pos.y - 2 * _radius - 1);
			// tmp - new line start point
            if (IsValid(binary, tmp, _radius) == 1) {
                UpdatePath(binary, tmp, _path, trace, _radius);

				if(search_dir_change){
					insert_a_star_path(_binary, _path, _radius);
					search_dir_change = false;	
				}
				inst_before_up = true;

                continue;
            }else {
				if(inst_before_up) {
					search_dir_change = true;
					inst_before_up = false;
				}
			}
        }

		// if search up is finished, search down
		
        //  search down
        if (trace.top().m_checkDown != true) {
            trace.top().m_checkDown = true;
            Point tmp(trace.top().m_pos.x, trace.top().m_pos.y + 2 * _radius + 1);
            if (IsValid(binary, tmp, _radius)) {
                UpdatePath(binary, tmp, _path, trace, _radius);

				if(search_dir_change){
					insert_a_star_path(_binary, _path, _radius);
					search_dir_change = false;	
				}
				inst_before_down = true;

                continue;
            }else {
				if(inst_before_down) {
					search_dir_change = true;
					inst_before_down = false;
				}
			}
        }

        trace.pop();
    }

	insert_a_star_path(_binary, _path, _radius, true, _goal);

    return 0;
}
