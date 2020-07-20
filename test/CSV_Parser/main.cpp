#include "CSVParser.h"

//#define FILE_PATH "../../dataset/MH_01_easy/mav0/cam0/data.csv"
#define FILE_PATH "data.csv"

int main()
{
    std::ifstream file(FILE_PATH);
    CSVParser row;
    while (file >> row)
    {
        std::cout << "Time Stamp [ns] : " << row[0] << " ";
        std::cout << "File Name : " << row[1] << "\n";
    }
}