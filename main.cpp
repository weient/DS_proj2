#include <iostream>
#include <queue>
#include <string>
#include <fstream>
//#include <time.h>
using namespace std;

double time_start, time_end;
class path_node{
public:
    int r, c;
    path_node* next;
    path_node(int ir = -1, int ic = -1): r(ir), c(ic), next(NULL){}
    path_node(const path_node &tmp): next(NULL){
        r = tmp.r;
        c = tmp.c;
    }
    void operator = (const path_node &ori){
        this->r = ori.r;
        this->c = ori.c;
        this->next = ori.next;
    }
    ~path_node(){}
};
int** bfs_map;
int** visited;
path_node* list;
path_node* global_cur;
int step_sum;
int cur_step;
int row, col, battery;
int init_r, init_c;
int mov_x[4] = {0, 0, 1, -1};
int mov_y[4] = {1, -1, 0, 0};
char** matrix;
fstream in_file;
fstream out_file;


void init_map(){
    queue<path_node> q;
    q.push(path_node(init_r, init_c));
    while(!q.empty()){
        path_node tmp = q.front();
        q.pop();
        for(int i=0; i<4; i++){
            if(tmp.r+mov_y[i]<0 || tmp.r+mov_y[i]>=row || tmp.c+mov_x[i]<0 || tmp.c+mov_x[i]>=col || bfs_map[tmp.r+mov_y[i]][tmp.c+mov_x[i]] != -2){
                continue;
            }
            q.push(path_node(tmp.r+mov_y[i], tmp.c+mov_x[i]));
            bfs_map[tmp.r+mov_y[i]][tmp.c+mov_x[i]] = bfs_map[tmp.r][tmp.c] + 1;
        }
    }
    
    
}
void back_route(int cur_r, int cur_c){
    int back_r = cur_r;
    int back_c = cur_c;
    cur_step = bfs_map[cur_r][cur_c];
    step_sum += (bfs_map[cur_r][cur_c]*2);
    path_node* reverse_path = new path_node(cur_r, cur_c);
    path_node* cur = NULL;
    path_node* last = reverse_path;
    while(bfs_map[back_r][back_c] != 0){
        for(int i=0; i<4; i++){
            int next_r = back_r+mov_y[i];
            int next_c = back_c+mov_x[i];
            if(next_r < row && next_r >= 0 && next_c < col && next_c >= 0){
                if(bfs_map[next_r][next_c] != -1 && bfs_map[next_r][next_c] < bfs_map[back_r][back_c]){
                    global_cur->next = new path_node(next_r, next_c);
                    global_cur = global_cur->next;
                    back_r = next_r;
                    back_c = next_c;
                    cur = new path_node(next_r, next_c);
                    cur->next = last;
                    last = cur;
                }
            }
        }
    }
    global_cur->next = cur->next;
    global_cur = reverse_path;
}
void robot_walk_walk(int last_r, int last_c, int cur_r, int cur_c){
    //come to a new position
    if(cur_r == init_r && cur_c == init_c){
        cur_step = 0;
        step_sum = 0;
    }
    else {
        cur_step++;
        step_sum++;
    }
    global_cur->next = new path_node(cur_r, cur_c);
    global_cur = global_cur->next;
    visited[cur_r][cur_c] = 1;
    for(int i=0; i<4; i++){
        int next_r = cur_r + mov_y[i];
        int next_c = cur_c + mov_x[i];
        if(next_r < row && next_r >= 0 && next_c < col && next_c >= 0)
            if(!visited[next_r][next_c]){
                if(bfs_map[next_r][next_c]+cur_step+1 > battery){
                    back_route(cur_r, cur_c);
                }
                robot_walk_walk(cur_r, cur_c, next_r, next_c); //recursive next step
                //come back to this position
                if(cur_r == init_r && cur_c == init_c) cur_step = 0;
                else cur_step++;
                step_sum++;
                global_cur->next = new path_node(cur_r, cur_c);
                global_cur = global_cur->next;
            }
    }
    if(last_r == -1) return;
    if(bfs_map[last_r][last_c]+cur_step+1 > battery){
        back_route(cur_r, cur_c);
    }
}


int main(int argc, char *argv[]){
    string filename(argv[1]);
    in_file.open(filename, ios::in);
    if(!in_file) {
        cout << "Can't open in_file!\n";
    }

    //time_start = clock();

    in_file >> row >> col >> battery;
    //new一個空的地圖
    //一個空的visited map
    matrix = new char*[row];
    visited = new int*[row];
    bfs_map = new int*[row];
    for(int i=0; i<row; i++){
        matrix[i] = new char[col];
        visited[i] = new int[col];
        bfs_map[i] = new int[col];
    }
    //讀進檔案，找到R的位置。
    //初始化visited map
    for(int i=0; i<row; i++){
        for(int j=0; j<col; j++){
            in_file >> matrix[i][j];
            if(matrix[i][j] == ' ' || matrix[i][j] == '\n')
                in_file >> matrix[i][j];
            if(matrix[i][j] == 'R'){
                init_r = i;
                init_c = j;
                visited[i][j] = 2;
                bfs_map[i][j] = 0;
            }
            else if(matrix[i][j] == '1'){
                visited[i][j] = -1;
                bfs_map[i][j] = -1;
            }
            else{
                visited[i][j] = 0;
                bfs_map[i][j] = -2;
            } 
        }
    }
    in_file.close();
    init_map();
    list = new path_node();
    global_cur = list;
    robot_walk_walk(-1, -1, init_r, init_c);
    //cout << step_sum << endl;
    list = list->next;
    //
    int flag = 0;
    int count = 0;
    int last_r = list->r, last_c = list->c;
    
    out_file.open("final.path", ios::out);
    if(!out_file) {
        cout << "Can't open out_file!\n";
    }
    out_file << step_sum << "\n";
    while(list != NULL){
        count++;
        out_file << list->r << " " << list->c << endl;
        list = list->next;
        //檢查答案
        if(list!=NULL && last_r != list->r && last_c != list->c) flag = 1;
        if(list!=NULL){
            last_r = list->r;
            last_c = list->c;
        }
    }
    out_file.close();
    //time_end = clock();
    //跑code時間
    //cout << "total time: " << (time_end - time_start) / CLOCKS_PER_SEC << " s" << endl;
    //檢查答案
    //if(count-1!=step_sum) flag = 1;
    //for(int i=0; i<row; i++){
    //    for(int j=0; j<col; j++){
    //        if(visited[i][j] == 0)
    //            flag = 1;
    //    }
    //}
    //有三種可能：總步數不相符合、每一步不連貫、沒有每格都走到
    //if(flag) cout << "wrong!!!\n";
    return 0;
}
