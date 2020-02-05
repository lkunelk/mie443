// C++ program to find the shortest path between 
// a given source cell to a destination cell. 
#include <bits/stdc++.h> 
using namespace std; 
#define ROW 9 
#define COL 10 

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
bool isValid(int row, int col) 
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
    cout << "print path";
    int size = path.size(); 
    for (int i = 0; i < size; i++)  
        cout << path[i].pt.x << "," << path[i].pt.y << " ";     
    cout << endl; 
} 

// function to find the shortest path between 
// a given source cell to a destination cell. 
int BFS(int mat[][COL], Point src, Point dest) 
{ 
	// check source and destination cell 
	// of the matrix have value 1 
	if (!mat[src.x][src.y] || !mat[dest.x][dest.y]) 
		return -1; 

	bool visited[ROW][COL]; 
	memset(visited, false, sizeof visited); 
	
	// Mark the source cell as visited 
	visited[src.x][src.y] = true; 

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

		queueNode curr = path[path.size() - 1]; 
		Point pt = curr.pt; 


		// If we have reached the destination cell, 
		// we are done 
		// if (mat[pt.x][pt.y] == 20 || pt.x == dest.x && pt.y == dest.y) 
		if (mat[pt.x][pt.y] == 20) 
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

			
			// if adjacent cell is valid, has path and 
			// not visited yet, enqueue it. 
			if (isValid(row, col) && mat[row][col] && 
			!visited[row][col]) 
			{ 
				// mark cell as visited and enqueue it 
				visited[row][col] = true; 
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
	int mat[ROW][COL] = 
	{ 
		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 }, 
		{ 1, 0, 1, 0, 1, 1, 1, 0, 1, 1 }, 
		{ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 }, 
		{ 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }, 
		{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 }, 
		{ 1, 0, 1, 1, 20, 1, 0, 1, 0, 0 }, 
		{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 }, 
		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 }, 
		{ 1, 1, 0, 0, 0, 0, 1, 0, 0, 1 } 
	}; 

	Point source = {0, 0}; 
	Point dest = {3, 4}; 

	int dist = BFS(mat, source, dest); 

	if (dist != INT_MAX) 
		cout << "Shortest Path is " << dist ; 
	else
		cout << "Shortest Path doesn't exist"; 

	return 0; 
} 
