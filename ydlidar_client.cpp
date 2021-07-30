/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

// 기존 코드
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#define RAD2DEG(x) ((x)*180./M_PI)

// 수정 코드
#include <vector>
#include <deque>
#include <algorithm>
#define DEQUE_SIZE 6 // deque 크기 설정 : 0.125초 단위로 갱신
#define DIV 1 // 몇 도 단위로 데이터를 기록할 것인가
#define SIZE 360/DIV // 나누어진 각도 단위
#define DIR 5 // 구간 나누기 : 좌측, 좌측 후방, 후방 정면, 우측 후방, 우측
#define DIST 9 // 거리 나누기 : 1m, 2m, 3m, 4m, 5m, 6m, 7m, 8m, 9m
std::string dir_str[DIR] = {"좌측", "좌측 후방", "후방 정면", "우측 후방", "우측"}; // 방향 문자열
std::string dist_str[DIST] = {"1m 이내", "2m 이내", "3m 이내", "4m 이내", "5m 이내", "6m 이내", "7m 이내", "8m 이내", "9m 이내"}; // 거리 문자열
std::deque<std::vector<std::vector<int>>> dng_dir; // 위험 정보 [저장순서][방향][거리]
std::deque<std::vector<std::vector<int>>> dng_dir_cnt; // 위험 정보 카운트 [저장순서][방향][거리]
std::deque<std::vector<float>> position; // 위치정보 [저장순서][각도]
std::deque<std::vector<float>> velocity; // 속도정보 [저장순서][각도]
std::deque<std::vector<float>> acceleration; // 가속도정보 [저장순서][각도]
std::deque<ros::Duration> time_info; // 시각정보 [저장순서]
ros::Time begin; // 노드 시작 시간을 기록할 변수

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 기존 코드
    int count = scan->scan_time / scan->time_increment; // 스캔 빈도 계산
    //printf("\n[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    //printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    // 함수에서 사용할 변수 선언
    float cnt_sum = 0.; // 사용 가능한 데이터 개수
    float cnt_fail = 0.; // 사용 불가능한 데이터 개수
    std::vector<std::vector<int>> dir(DIR, std::vector<int>(DIST, 0)); // 임시 위험 정보 [방향][거리]
    std::vector<std::vector<int>> dir_cnt(DIR-1, std::vector<int>(DIST-1, 0)); // 임시 위험 정보 합 [방향][거리]
    std::vector<float> ang_dist(SIZE, 0); // 거리의 합
    std::vector<float> cnt(SIZE, 0); // 데이터 개수
    std::vector<float> mn_dist(SIZE, 0); // 거리의 평균 (거리의 합 / 데이터 개수)
    std::vector<float> ang_vel(SIZE, 0); // 속도 저장
    std::vector<float> ang_acl(SIZE, 0); // 가속도 저장
    ros::Duration time_tmp=scan->header.stamp - begin; // client 시작 이후로 경과한 시간

    // 오래된 데이터 삭제
    if (position.size()==DEQUE_SIZE) { // deque 크기 제한을 초과하면
        dng_dir.pop_front(); // 위험 정보 삭제
        dng_dir_cnt.pop_front(); // 위험 정보 합 삭제
        position.pop_front(); // 오래된 위치 정보 삭제
        velocity.pop_front(); // 오래된 속도 정보 삭제
        acceleration.pop_front(); // 오래된 가속도 정보 삭제
        time_info.pop_front(); // 오래된 시간 정보 삭제
    }

    // cnt(데이터 개수), ang_dist(거리 합) 처리
    for(int i = 0; i < count; i++) { // for문 돌면서 0이 아닌 값을 ang_dist에 저장
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i) + 180; // 각도 구하기
        if(scan->ranges[i] != 0.) { // 유효한 데이터일 경우
            cnt_sum ++; // 사용 가능한 데이터 카운트
            cnt[degree/DIV]++; // 각도에 해당되는 데이터 개수 카운트
            ang_dist[degree/DIV] += scan->ranges[i]; // 각도에 해당되는 거리 더해주기
        }
        else cnt_fail++; // 사용 불가능한 데이터 카운트
    }

    // mn_dist(거리 평균) 처리
    for(int i = 0; i < SIZE; i++) {
        if (cnt[i] !=0 ) mn_dist[i] = ang_dist[i]/cnt[i]; // 각도별 평균값 구하기
    }

    // 속도 정보 구하기
    if(position.size() > 0){ // 위치 정보가 하나 이상 저장되어 있을 경우
        for(int i = 0; i < SIZE; i++) {
            if (mn_dist[i] != 0 && position[position.size()-1][i] != 0){ // 사용 가능한 데이터일 경우
                float t = time_tmp.toSec() - time_info[position.size()-1].toSec(); // 시간 차이 계산
                ang_vel[i] = (position[position.size()-1][i] - mn_dist[i]) / t; // 각도별 속도 구하기
            }
        }
    }

    // 가속도 정보 구하기
    if(velocity.size() > 1){ // 속도 정보가 하나 이상 저장되어 있을 경우
        for(int i = 0; i < SIZE; i++) {
            if (ang_vel[i] !=0 && velocity[velocity.size()-1][i] != 0) { // 사용 가능한 데이터일 경우
                float t = time_tmp.toSec() - time_info[position.size()-1].toSec(); // 시간 차이 계산
                ang_acl[i] = (ang_vel[i] - velocity[velocity.size()-1][i]) / t; // 각도별 가속도 구하기
            }
        }
    }

    // 필터링
    for(int i = 0; i < SIZE; i++) {
        if (ang_vel[i] < 3.5 && ang_acl[i] < 2.5 && ang_acl[i] > 0) { // 튀는 값 걸러내기
            int ang_tmp = i*DIV - 40;
            if (ang_tmp >= 0 && ang_tmp < 270){
                ang_tmp /= 270/DIR;
                if (ang_vel[i] > 0.5 && mn_dist[i] < 1) { // 속도 0.5m/s 이상 거리 1m 이내
                    dir[ang_tmp][0]++;
                }
                else if (ang_vel[i] > 0.6 && mn_dist[i] < 2) { // 속도 0.6m/s 이상 거리 2m 이내
                    dir[ang_tmp][1]++;
                }
                else if (ang_vel[i] > 0.7 && mn_dist[i] < 3) { // 속도 0.7m/s 이상 거리 3m 이내
                    dir[ang_tmp][2]++;
                }
                else if (ang_vel[i] > 0.8 && mn_dist[i] < 4) { // 속도 0.8m/s 이상 거리 4m 이내
                    dir[ang_tmp][3]++;
                }
                else if (ang_vel[i] > 0.9 && mn_dist[i] < 5) { // 속도 0.9m/s 이상 거리 5m 이내
                    dir[ang_tmp][3]++;
                }
                else if (ang_vel[i] > 1.0 && mn_dist[i] < 6) { // 속도 1.0m/s 이상 거리 6m 이내
                    dir[ang_tmp][3]++;
                }
                else if (ang_vel[i] > 1.1 && mn_dist[i] < 7) { // 속도 1.1m/s 이상 거리 7m 이내
                    dir[ang_tmp][3]++;
                }
                else if (ang_vel[i] > 1.2 && mn_dist[i] < 8) { // 속도 1.2m/s 이상 거리 8m 이내
                    dir[ang_tmp][3]++;
                }
                else if (ang_vel[i] > 1.3 && mn_dist[i] < 9) { // 속도 1.3m/s 이상 거리 9m 이내
                    dir[ang_tmp][3]++;
                }
            }
            for(int j=0; j<DIR-1; j++){
                for(int k=0; k<DIST-1; k++){
                    dir_cnt[j][k] += dir[j][k] + dir[j+1][k] + dir[j][k+1] + dir[j+1][k+1];
                }
            }
            //printf("ang pos vel acl time : %3d, %9.6f %9.6f %9.6f %f\n", i*DIV, mn_dist[i], ang_vel[i], ang_acl[i], time_tmp.toSec());
        }
    }

    // 위험, 위치, 속도, 가속도, 시간 정보 갱신
    dng_dir.push_back(dir); // 위험 정보 갱신
    dng_dir_cnt.push_back(dir_cnt); // 위험 정보 합 갱신
    position.push_back(mn_dist); // 위치 정보 갱신
    velocity.push_back(ang_vel); // 속도 정보 갱신
    acceleration.push_back(ang_vel); // 가속도 정보 갱신
    time_info.push_back(time_tmp); // 시간 정보 갱신

    // 덱에 쌓인 위험 신호 체크
    std::vector<std::vector<int>> counting_danger(DIR-1, std::vector<int>(DIST-1, 0)); // 덱에 쌓인 위험 신호 카운트
    for (int i=0; i<dng_dir_cnt.size(); i++){
        for (int j=0; j<DIR-1; j++){
            for (int k=0; k<DIST-1; k++){
                if (dng_dir_cnt[i][j][k] > 0) counting_danger[j][k]++;
                if (counting_danger[j][k] == DEQUE_SIZE-1){
                    for (int m=j; m<j+2; m++){
                        for (int n=k; n<k+2; n++){
                            if (dir[m][n] > 0) std::cout << dir_str[m] << " " << dist_str[n] << " " << dir[m][n] << "개 각도에서 위험 감지!!\n";
                        }
                    }
                }
            }
        }
    }

    /*// 축적한 데이터 출력하기
    for(int i = 0; i < position.size(); i++) {
        for (int j = 0; j < SIZE; j++){
            if (j%3==0) printf("\n");
            printf("ang pos vel acl : %3d %9.6f %9.6f %9.6f   ", j*DIV, position[i][j], velocity[i][j], acceleration[i][j]);
        }
        std::cout << "\n" << "deque num : " << i << ", time_info : " << time_info[i];
        if (i>0) std::cout << ", time_inc : " << time_info[i]-time_info[i-1];
    }*/

    //printf("\n[YDLIDAR INFO]: count, cnt_sum + cnt_fail, cnt_sum, cnt_fail : %d, %.0f, %.0f, %.0f\n", count, cnt_sum + cnt_fail, cnt_sum, cnt_fail);
}

int main(int argc, char **argv)
{
    // 노드 초기 설정
    printf("node start...\n");
    ros::init(argc, argv, "ydlidar_client"); // 노드 초기화
    ros::NodeHandle n; // 노드 핸들 선언
    begin = ros::Time::now(); // 노드 시작 시간 기록
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback); // 데이터 받으면 callback 함수 호출
    ros::spin(); // 프로그램을 종료하지 않도록 유지

    return 0;
}
