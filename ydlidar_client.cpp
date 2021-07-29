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
#define DIV 1 // 몇 도 단위로 데이터를 기록할 것인가
#define DIR 5 // 구간 나누기
#define SIZE 360/DIV // 나누어진 각도 단위
#define DEQUE_SIZE 4 // deque 크기 설정
std::deque<std::vector<int>> dng_dir; // 위험 정보
std::vector<int> dir_sum(DIR, 0); // 위험 정보 합
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
    std::vector<int> dir(DIR, 0); // 임시 위험 정보
    std::vector<float> ang_dist(SIZE, 0); // 거리의 합
    std::vector<float> cnt(SIZE, 0); // 데이터 개수
    std::vector<float> mn_dist(SIZE, 0); // 거리의 평균 (거리의 합 / 데이터 개수)
    std::vector<float> ang_vel(SIZE, 0); // 속도 저장
    std::vector<float> ang_acl(SIZE, 0); // 가속도 저장
    ros::Duration time_tmp=scan->header.stamp - begin; // client 시작 이후로 경과한 시간
    
    // 오래된 데이터 삭제
    if (position.size()==DEQUE_SIZE) { // deque 크기 제한을 초과하면
        for(int i = 0; i < DIR; i++) {dir_sum[i] -= dng_dir[0][i];}
        dng_dir.pop_front(); // 위험 정보 삭제
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
        if (ang_vel[i] > 0.5 && ang_vel[i] < 3.5 && ang_acl[i] < 3.5 && ang_acl[i] > 0) {
        //if (i*DIV < 190 && i*DIV > 155) {
            int ang_tmp = i*DIV - 40;
            if (ang_tmp >= 0 && ang_tmp < 270){
                dir[ang_tmp / 54]++;
                dir_sum[ang_tmp / 54]++;
            }
            //printf("ang pos vel acl time : %3d, %9.6f %9.6f %9.6f %f\n", i*DIV, mn_dist[i], ang_vel[i], ang_acl[i], time_tmp.toSec());
        }
    }

    // 위험, 위치, 속도, 가속도, 시간 정보 갱신
    dng_dir.push_back(dir); // 위험 정보 갱신
    position.push_back(mn_dist); // 위치 정보 갱신
    velocity.push_back(ang_vel); // 속도 정보 갱신
    acceleration.push_back(ang_vel); // 가속도 정보 갱신
    time_info.push_back(time_tmp); // 시간 정보 갱신

    // 위험 정보 총 합
    printf("왼쪽 : %3d, 왼쪽뒤 : %3d, 뒤쪽 : %3d, 오른쪽뒤 : %3d, 오른쪽 : %3d\n", dir_sum[0], dir_sum[1], dir_sum[2], dir_sum[3], dir_sum[4]);

    /*// 축적한 데이터 출력하기
    for(int i = 0; i < position.size(); i++) {
        for (int j = 0; j < SIZE; j++){
            if (j%3==0)) {
                printf("ang pos vel acl : %3d %9.6f %9.6f %9.6f   ", j*DIV, position[i][j], velocity[i][j], acceleration[i][j]);
            }
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
