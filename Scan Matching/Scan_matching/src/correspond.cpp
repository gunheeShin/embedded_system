#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "math.h"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;


void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){
      
      c.clear();                        
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        min_dist = 100000.00;
        for(int j = 0; j<m; ++j){
          float dist = old_points[j].distToPoint2(&trans_points[i]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void getSmartJumpCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob, float incre,int& jump_index){

  c.clear();

  int last_best = -1;
  const int m = trans_points.size();
  const int n = old_points.size();
  vector <int> up_to_down; //debuging vector
  jump_index=0;


  for(int i = 0; i<m; ++i){

    up_to_down.clear();

    int best = -1;
    int second_best = -1;

    double best_dis = old_points[last_best].distToPoint2(&trans_points[i]);
    double  point_dis = trans_points[i].r; 
    double  point_ang = trans_points[i].theta+M_PI;

    double up_theta_jump =-1;
    double down_theta_jump=-1;

    int start_index = int(point_ang/incre);
    int opposite_start_index = start_index+540 > 1079 ? start_index-540:start_index+540;

    int we_start_at = start_index; 

    int up_check = we_start_at+1;
    int down_check = we_start_at;
    if(up_check==1080) up_check--;

    double up_check_dis = -1;
    double down_check_dis = -2;

    double del_theta_down = -1;
    double del_theta_up=-1;

    bool up_stopped=false, down_stopped=false;
    bool up_out = false;
    bool down_out = false;

    up_to_down.push_back(last_best);

    while(!(up_stopped && down_stopped)){

      jump_index++;

      // bool now_up = !up_stopped;
      bool now_up = up_stopped ? 0 :
                  down_stopped ? 1 : up_check_dis<down_check_dis;

      if(now_up){                                                               // Searching up direction

        up_to_down.push_back(up_check);
        
        if(!up_out&&(up_check >= n)){                                           // Set the up_check index to 0 when the index is out of full range(n or 1080)
          up_out = true;
          up_check = 0;
          continue;
        }

        if(up_out&&(up_check>opposite_start_index)){                            // Jump_table algorithm is vaild from start_index to opposite_start_index over up direction
          up_stopped=true; continue;
        }

        up_check_dis = old_points[up_check].distToPoint2(&trans_points[i]);
        if(up_check_dis=best_dis) { best = up_check; best_dis = up_check_dis;}  // If up_check_dis is shorter than current best_dis, update best_dis to up_check_dis and best index to up_check index
        
        del_theta_up = abs(point_ang-(up_check)*incre);                         // Calculate min_dist_up using del_theta_up. 
        if(del_theta_up > M_PI){del_theta_up = 2*M_PI-del_theta_up;}            // if min_dist_up is bigger than current best_dis, stop searching up direction
        del_theta_up = del_theta_up>=0.5*M_PI ? 0.5*M_PI : del_theta_up;
        double min_dist_up = abs(sin(del_theta_up)*point_dis);
        if(pow(min_dist_up,2)>best_dis){
          up_stopped=true; 
          continue;
        }

        double inverse_val_up = (up_check_dis+pow(old_points[up_check].r,2)-pow(point_dis,2))/(2*sqrt(up_check_dis)*old_points[up_check].r);
        if(inverse_val_up<-1){inverse_val_up=-1;}                               // Calculate up_theta_jump using inverse_val_up
        else if(inverse_val_up>1){inverse_val_up=1;}
        up_theta_jump=acos(inverse_val_up);

        if(up_theta_jump>0.5*M_PI){                                             // If up_theta_jump is bigger than pi/2, next up check index would be UP_BIG of current up_check index
          up_check = jump_table[up_check][UP_BIG]; up_to_down.push_back(-3);    
        }else if(up_theta_jump<0.5*M_PI){
          up_check = jump_table[up_check][UP_SMALL]; up_to_down.push_back(-2);  // If up_theta_jump is smaller than pi/2, next up check index would be UP_SMALL of current up_check index
        }else{ROS_INFO("up_check_dis : %f, 0tochecking point^2 : %f, point_dis^2 : %f, 2*a*b : %f", 
        up_check_dis, pow(old_points[up_check].r,2),pow(point_dis,2), 2*sqrt(up_check_dis)*old_points[up_check].r);}
      }
      
      else{                                                                     // Searching down direction
        up_to_down.push_back(down_check);

        if(!down_out&&(down_check < 0)){                                        // Set the down_check index to 1079 when the index is out of full range(0)
          down_out=true;
          down_check=1079;
        }

        if(down_out&&(down_check<opposite_start_index)){                        // Jump_table algorithm is vaild from start_index to opposite_start_index over down direction
          down_stopped=true; continue;
        }
        
        down_check_dis = old_points[down_check].distToPoint2(&trans_points[i]); // If down_check_dis is shorter than current best_dis, update best_dis to down_check_dis and best index to down_check index
        if((down_check_dis<=best_dis)) {best = down_check; best_dis = down_check_dis;}

        del_theta_down = abs(point_ang-(down_check)*incre);                     // Calculate min_dist_down using del_theta_down.
        if(del_theta_down > M_PI){del_theta_down = 2*M_PI-del_theta_down;}      // if min_dist_down is bigger than current best_dis, stop searching down direction
        del_theta_down = del_theta_down>=0.5*M_PI ? 0.5*M_PI : del_theta_down;
        double min_dist_down = abs(sin(del_theta_down)*point_dis);
        if(pow(min_dist_down,2)>best_dis){
            down_stopped=true; 
            continue;
        }

        double inverse_val_down = (down_check_dis+pow(old_points[down_check].r,2)-pow(point_dis,2))/(2*sqrt(down_check_dis)*old_points[down_check].r);
        if(inverse_val_down<-1){inverse_val_down=-1;}                           // Calculate down_theta_jump using inverse_val_down
        else if(inverse_val_down>1){inverse_val_down=1;}
        down_theta_jump=acos(inverse_val_down);
        
        if(down_theta_jump>0.5*M_PI){                                           // If down_theta_jump is bigger than pi/2, next up check index would be DOWN_BIG of current down_check index
          down_check = jump_table[down_check][DOWN_BIG]; up_to_down.push_back(-5);
        }else if(down_theta_jump<0.5*M_PI){                                     // If down_theta_jump is smaller than pi/2, next up check index would be DOWN_SMALL of current down_check index
          down_check = jump_table[down_check][DOWN_SMALL]; up_to_down.push_back(-4);
        }else{ROS_INFO("down_check_dis : %f, 0tochecking point^2 : %f, point_dis^2 : %f, 2*a*b : %f", 
        down_check_dis, pow(old_points[down_check].r,2),pow(point_dis,2), 2*sqrt(down_check_dis)*old_points[down_check].r);}
      }
    }

    last_best = best;
    second_best = last_best-1;
    if(second_best<0){second_best=last_best+1;}

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  
  }
}

void getSmartCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre, int& smart_index){
  
  c.clear();

  int last_best = -1;
  double prev_point_ang=-1.0;
  const int m = trans_points.size();
  const int n = old_points.size();
  int low_idx;
  int high_idx;
  smart_index = 0;
  vector <int> up_to_down;

  //Do for each point
  for(int i = 0; i<m; ++i){
    double best_dis = 10000000.00;
    int best = -1;
    int second_best = -1;

    if(last_best == -1){                                                  // At first, set best_dis and best index using naive correspondece algorithm
      for(int j=0;j<n; ++j){
        double dis = old_points[j].distToPoint2(&trans_points[i]);
        if(dis<best_dis){
          best_dis = dis;
          best = j;
          second_best = j-1;
          if(second_best == -1){
            second_best = 1;
          }
          last_best = best;
        }
      }
    }

    else{
      
      double  point_dis = trans_points[i].r;
      double  point_ang = trans_points[i].theta+M_PI;
      
      double  last_best_dis = old_points[last_best].distToPoint(&trans_points[i]);
      double  last_angle = 0;
      
      if (point_dis*point_dis - last_best_dis*last_best_dis > 0){         // When the origin(Lidar position) is located in the circle whose radius is last_best_dis 
        last_angle =  atan2(last_best_dis, sqrt(point_dis*point_dis - last_best_dis*last_best_dis));
        low_idx = int((point_ang - last_angle)/incre)-2; 
        high_idx = int((point_ang + last_angle)/incre)+2;
      }

      else{                                                               // When the origin(Lidar position) isn't located in the circle whose radius is last_best_dis 
        last_angle = M_PI;
        low_idx = int((point_ang - last_angle)/incre); 
        high_idx = int((point_ang + last_angle)/incre);
      }

      smart_index += high_idx-low_idx;
      
      if(low_idx<0) low_idx += 1080;
      if(high_idx>=n) high_idx -=1080;
      
      double dis = 0;
      if(high_idx>low_idx){                                               // When high_idx is bigger than low_idx, search the best index from low_idx to high_idx
        for(int j=low_idx; j<=high_idx; ++j){
          dis= old_points[j].distToPoint2(&trans_points[i]);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
          }
        }
      }
      
      else{                                                               // When high_idx is smaller than low_idx, first search the best index from 0 to high_idx
        for(int j=0;j<=high_idx;++j){
          dis= old_points[j].distToPoint2(&trans_points[i]);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
            }
        } 
        
        for(int j=low_idx;j<1080;++j){                                    // then search from low_idx to 1079
          dis= old_points[j].distToPoint2(&trans_points[i]);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
            }
        }  
      
      }
    
    }

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  }

}

void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>=points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<=points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>=points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}