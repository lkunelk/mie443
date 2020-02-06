// C++ program to find the shortest path between 
// a given source cell to a destination cell. 
#include <bits/stdc++.h> 
using namespace std; 

//To store matrix cell cordinates 
struct Point 
{ 
	int x; 
	int y; 
}; 

// A Data Structure for queue used in BFS 
struct queueNode 
{ 
	Point pt; // The cordinates of a cell 
	int dist; // cell's distance of from the source 
}; 

// check whether given cell (row, col) is a valid 
// cell or not. 
bool isValid(int row, int col, int ROW, int COL) 
{ 
	// return true if row number and column number 
	// is in range 
	return (row >= 0) && (row < ROW) && 
		(col >= 0) && (col < COL); 
} 

// These arrays are used to get row and column 
// numbers of 4 neighbours of a given cell 
int rowNum[] = {-1, 0, 0, 1}; 
int colNum[] = {0, -1, 1, 0}; 

void printpath(vector<queueNode>& path) 
{ 
    cout << "Path: ";
    int size = path.size(); 
    for (int i = 0; i < size; i++)  
        cout << path[i].pt.x << "," << path[i].pt.y << " ";     
    cout << endl; 
} 

int convert2Dto1D(int row, int col, int numCol)
{
    return col + numCol * row;
}

bool checkNeighboursWithDepth(int mat[], int range, int x, int y, int numRow, int numCol)
{ 
    for(int i = -range; i <= range; i++){
        for(int j = -range; j <= range; j++){
            int row = x + i; 
			int col = y + j;
            cout << row << ", " << col << " :" << mat[convert2Dto1D(row, col, numCol)];
            cout << endl;
            
            if (!isValid(row, col, numRow, numCol) || mat[convert2Dto1D(row, col, numCol)] != -1 )
			{ 
                return false;
            }

        }
    }


	// Return -1 if destination cannot be reached 
	return true; 
}

// function to find the shortest path between 
// a given source cell to a destination cell. 
vector<queueNode>  BFS(int mat[], int numRow, int numCol, Point src) 
{ 

    vector<queueNode> path; 

	// check source and destination cell 
	// of the matrix have value 1 
	// if (mat[convert2Dto1D(src.x, src.y, numCol)]) 
	// 	return path; 

	bool visited[convert2Dto1D(numRow, numCol, numCol) - 1]; 
	memset(visited, false, sizeof visited); 
	
	// Mark the source cell as visited 
	visited[convert2Dto1D(src.x, src.y, numCol)] = true; 

	// Create a queue for BFS 
	queue<vector<queueNode> > q; 

	// Distance of source cell is 0 
	queueNode s = {src, 0}; 

    path.push_back(s); 	

	q.push(path); // Enqueue source cell 

	// Do a BFS starting from source cell 
	while (!q.empty()) 
	{ 
        path = q.front();

		queueNode curr = path[path.size() - 1]; 
		Point pt = curr.pt; 


		// If we have reached the destination cell, 
		// we are done 
		// if (mat[pt.x][pt.y] == 20 || pt.x == dest.x && pt.y == dest.y) 
		
        // cout << convert2Dto1D(pt.x, pt.y, numCol) << " ";
        if (mat[convert2Dto1D(pt.x, pt.y, numCol)] == -1 && path.size() >= 1) 
        {
            // check a (2 X depth + 1) square grid around a point. e.g. 2 = 5X5 grid
            if (checkNeighboursWithDepth(mat, 2, pt.x, pt.y, numRow, numCol)){
                return path; 
            }
        }

        q.pop();

		// Otherwise dequeue the front cell in the queue 
		// and enqueue its adjacent cells 
		// q.pop(); 

		for (int i = 0; i < 4; i++) 
		{ 
			int row = pt.x + rowNum[i]; 
			int col = pt.y + colNum[i]; 

			// cout << row << "," << col << "," << convert2Dto1D(row, col, numCol);
			// cout << "valid?:" << isValid(row, col, numRow, numCol);
            // cout << endl; 

			// if adjacent cell is valid, has path and 
			// not visited yet, enqueue it. 
			if (isValid(row, col, numRow, numCol) && mat[convert2Dto1D(row, col, numCol)] < 50 && 
			!visited[convert2Dto1D(row, col, numCol)]) 
			{ 
				// mark cell as visited and enqueue it 
                // cout << row << "," << col << "," << convert2Dto1D(row, col, numCol);
				visited[convert2Dto1D(row, col, numCol)] = true; 
				queueNode Adjcell = { {row, col}, 
									curr.dist + 1 }; 


                vector<queueNode> newpath(path);
                newpath.push_back(Adjcell); 


				q.push(newpath); 
			} 
		} 
	} 

    vector<queueNode> nopath; 
	// Return -1 if destination cannot be reached 
	return nopath; 
} 

// Driver program to test above function 
int main() 
{ 
    int ROW = 8;
    int COL = 7;
	int mat[convert2Dto1D(ROW, COL, COL) - 1] = 
	{  
		  0,  -1, -1,    0, 0, 1, -1,
          -1, -1,  0,   -1,  0,  1, -1,
          0,  -1,  -1,  -1, -1, 0, -1,
          1,   -1, -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1
	}; 

	Point source = {0, 0}; 

	vector<queueNode> path = BFS(mat, ROW, COL, source); 

    printpath(path);
    queueNode curr = path[path.size() - 1]; 
    cout << curr.pt.x << curr.pt.y;

	// if (dist != INT_MAX) 
	// 	cout << "Shortest Path is " << dist ; 
	// else
	// 	cout << "Shortest Path doesn't exist"; 

	return 0; 
} 
