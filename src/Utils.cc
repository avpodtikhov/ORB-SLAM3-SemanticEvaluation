//
// Created by fontan on 9/06/23.
//

#include "Utils.h"
#include <ctime>
#include <random>
#include <algorithm>

bool ORB_SLAM3::RandomIntegerGenerator::need_to_be_seeded = true;
int ORB_SLAM3::RandomIntegerGenerator::randomIntegerIndex = -1;
std::vector<int> ORB_SLAM3::RandomIntegerGenerator::listOfRandomIntegers{};

std::string ORB_SLAM3::paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}

void ORB_SLAM3::RandomIntegerGenerator::seedRandomGenerator(){
    if(need_to_be_seeded){
        srand (0);
        need_to_be_seeded = false;
        listOfRandomIntegers.reserve(amountOfRandomIntegers);
        for(int iRand{}; iRand < amountOfRandomIntegers; iRand++)
            listOfRandomIntegers.push_back(rand());
    }
}

int ORB_SLAM3::RandomIntegerGenerator::getRandomNumber(){
    seedRandomGenerator();
    randomIntegerIndex++;
    if(randomIntegerIndex >= listOfRandomIntegers.size()){
        std::shuffle ( listOfRandomIntegers.begin(), listOfRandomIntegers.end(), std::default_random_engine(0));
        randomIntegerIndex = 0;
    }
    return listOfRandomIntegers[randomIntegerIndex];
}

int ORB_SLAM3::RandomIntegerGenerator::getRandomNumber(const int& maxNumber) {
    return int((((double)getRandomNumber())/((double)RAND_MAX)) * maxNumber) ;
}