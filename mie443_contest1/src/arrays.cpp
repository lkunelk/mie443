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
    if(row == 2 && col == 3) {
        cout << "row 2 col 3: ";
        cout << (row >= 0) && (row < ROW) && 
		    (col >= 0) && (col < COL);
        cout << endl;
    }
         
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

// function to find the shortest path between 
// a given source cell to a destination cell. 
int BFS(int mat[], int numRow, int numCol, Point src) 
{ 
	// check source and destination cell 
	// of the matrix have value 1 
	if (!mat[convert2Dto1D(src.x, src.y, numCol)]) 
		return -1; 

	bool visited[convert2Dto1D(numRow, numCol, numCol) - 1]; 
	memset(visited, false, sizeof visited); 
	
	// Mark the source cell as visited 
	visited[convert2Dto1D(src.x, src.y, numCol)] = true; 

	// Create a queue for BFS 
	queue<vector<queueNode> > q; 

    vector<queueNode> path; 
	// Distance of source cell is 0 
	queueNode s = {src, 0}; 

    path.push_back(s); 	

	q.push(path); // Enqueue source cell 

	// Do a BFS starting from source cell 
	while (!q.empty()) 
	{ 
        path = q.front();
        // printpath(path);

		queueNode curr = path[path.size() - 1]; 
		Point pt = curr.pt; 


		// If we have reached the destination cell, 
		// we are done 
		// if (mat[pt.x][pt.y] == 20 || pt.x == dest.x && pt.y == dest.y) 
		
        // cout << convert2Dto1D(pt.x, pt.y, numCol) << " ";
        if (mat[convert2Dto1D(pt.x, pt.y, numCol)] == 4) 
        {
            // int test = curr.dist;
            printpath(path);
			return curr.dist; 
        }

        q.pop();

		// Otherwise dequeue the front cell in the queue 
		// and enqueue its adjacent cells 
		// q.pop(); 

		for (int i = 0; i < 4; i++) 
		{ 
			int row = pt.x + rowNum[i]; 
			int col = pt.y + colNum[i]; 

			cout << row << "," << col << "," << convert2Dto1D(row, col, numCol);
			cout << "valid?:" << isValid(row, col, numRow, numCol);
            cout << endl; 

			// if adjacent cell is valid, has path and 
			// not visited yet, enqueue it. 
			if (isValid(row, col, numRow, numCol) && mat[convert2Dto1D(row, col, numCol)] && 
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

	// Return -1 if destination cannot be reached 
	return -1; 
} 

// Driver program to test above function 
int main() 
{ 
    int ROW = 5;
    int COL = 5;
	int mat[convert2Dto1D(ROW, COL, COL) - 1] = 
	{  
		  1, 0, 1, 1, 1,
          1, 1, 1, 0, 1,
          0, 0, 0, 0, 1,
          1, 1, 1, 1, 1,
          1, 1, 1, 4, 0
	}; 

	Point source = {0, 0}; 

	int dist = BFS(mat, ROW, COL, source); 

	if (dist != INT_MAX) 
		cout << "Shortest Path is " << dist ; 
	else
		cout << "Shortest Path doesn't exist"; 

	return 0; 
} 
