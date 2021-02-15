#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>


//#define ROW 9
//#define COL 5

#define ROW 9
#define COL 10


#define PATH_LEFT    0
#define PATH_UP      1
#define PATH_DOWN    2
#define PATH_RIGHT   3


//To store matrix cell cordinates
typedef struct Points{
	int x;
	int y;
} Point;

// A Data Structure for queue used in BFS
typedef struct QueueNodes
{
	Point pt; // The cordinates of a cell
	uint8_t path_from_source[50];
	uint8_t path_size;
} QueueNode;

//===============================================================================================================
//****************************************************
//** linked list based implementation of queue      **
//****************************************************

// A linked list (LL) node to store a queue entry
struct QNode {
	QueueNode node;
	struct QNode* next;
};

// The queue, front stores the front node of LL and rear stores the
// last node of LL
struct Queue {
	struct QNode *front, *rear;
};

// A utility function to create a new linked list node.
struct QNode* newNode(QueueNode node){
	struct QNode* temp = (struct QNode*)malloc(sizeof(struct QNode));
	temp->node = node;
	temp->next = NULL;
	return temp;
}

// A utility function to create an empty queue
struct Queue* createQueue(){
	struct Queue* q = (struct Queue*)malloc(sizeof(struct Queue));
	q->front = q->rear = NULL;
	return q;
}

// The function to add a key k to q
void enQueue(struct Queue* q, QueueNode node){
	// Create a new LL node
	struct QNode* temp = newNode(node);

	// If queue is empty, then new node is front and rear both
	if (q->rear == NULL) {
		q->front = q->rear = temp;
		return;
	}

	// Add the new node at the end of queue and change rear
	q->rear->next = temp;
	q->rear = temp;
}

// Function to remove a key from given queue q
void deQueue(struct Queue* q){
	// If queue is empty, return NULL.
	if (q->front == NULL)
		return;

	// Store previous front and move front one node ahead
	struct QNode* temp = q->front;

	q->front = q->front->next;

	// If front becomes NULL, then change rear also as NULL
	if (q->front == NULL)
		q->rear = NULL;

	free(temp);
}

//=================================================================================================================


// check whether given cell (row, col) is a valid
// cell or not.
uint8_t isValid(int row, int col){
	// return 1 if row number and column number
	// is in range
	return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// These arrays are used to get row and column
// numbers of 4 neighbours of a given cell
int rowNum[4] = {-1, 0, 0, 1};
int colNum[4] = {0, -1, 1, 0};

// function to find the shortest path between
// a given source cell to a destination cell.
int BFS(int mat[][COL], Point src, Point dest , uint8_t *path){
	// check source and destination cell
	// of the matrix have value 1
	if (!mat[src.x][src.y] || !mat[dest.x][dest.y])
		return -1;

	uint8_t visited[ROW][COL];
	memset(visited, 0 , sizeof visited);

	uint8_t tmp_path[50];
	memset(tmp_path, 0 , sizeof tmp_path);

	uint8_t path_size = 0;

	// Mark the source cell as visited
	visited[src.x][src.y] = 1;

	// Create a queue for BFS
	struct Queue* q = createQueue();

	// Distance of source cell is 0
	QueueNode s = {src, {0}, 0};

	enQueue(q,s); // Enqueue source cell

	// Do a BFS starting from source cell
	while (!(q->front == NULL) ) {
		QueueNode curr = q->front->node;
		Point pt = curr.pt;



		// If we have reached the destination cell,
		// we are done
		if (pt.x == dest.x && pt.y == dest.y){
            memcpy(path, curr.path_from_source , sizeof curr.path_from_source);
            return curr.path_size;
        }



		// Otherwise dequeue the front
		// cell in the queue
		// and enqueue its adjacent cells
		deQueue(q);

		for (int i = 0; i < 4; i++){

			int row = pt.x + rowNum[i];
			int col = pt.y + colNum[i];



			// if adjacent cell is valid, has path and
			// not visited yet, enqueue it.
			if (isValid(row, col) && mat[row][col] && !visited[row][col]) {

				visited[row][col] = 1;
                memcpy(tmp_path, curr.path_from_source , sizeof tmp_path);
                tmp_path[curr.path_size] = i;


				path_size = curr.path_size + 1;


				QueueNode Adjcell = { {row, col}, *tmp_path , path_size };

				memcpy(Adjcell.path_from_source, tmp_path , sizeof tmp_path);


				Adjcell.path_size = path_size;


				enQueue(q,Adjcell);
			}
		}

	}

	return -1;
}



