// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    // 생성자 : 시작 시간 측정
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    // duration :: https://en.cppreference.com/w/cpp/chrono/duration
    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;  // duration 기준을 안정했다면, duration은 1초 기준으로 설정됨
        return elapsed_seconds.count() * 1000;  // 1 tick = 1초
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
