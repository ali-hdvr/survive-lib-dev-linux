#include <iostream>
#include <fstream>
#include <string>
#include "spree/interface.h"
#include "spree/poser.h"
#include "json/json.hpp"
#include <vector>
#include <sstream>
#include <strings.h>

using namespace std;

typedef nlohmann::json json;

json poseconfig_json;

fstream file_config;

const string filename_tracker_config = "data/tracker 3.0 LHR-8E2FB205.json";

void init_trackerconfig()
{
    file_config.open(filename_tracker_config, std::fstream::in);
    try
    {
        file_config >> poseconfig_json;

        cout << poseconfig_json << std::endl;
    }
    catch (const std::exception &exc)
    {
        std::cerr << exc.what();

        cout << "\nError in extracting " << filename_tracker_config << ".json \n";
    }

    Poser::fill_tracker_config(poseconfig_json);
}

int main()
{

    init_trackerconfig();

    vector<string> row;
    string line, word, temp;

    ifstream fin("data/raw_sync_sweep_4lh_correct.csv");
    if (fin.is_open())
    {
        while (getline(fin, line))
        {
            row.clear();
            // read an entire row and
            // store it in a string variable 'line'

            // used for breaking words
            stringstream s(line);

            // cout << "line: " << line << endl;
            // read every column data of a row and
            // store it in a string variable, 'word'
            while (getline(s, word, ','))
            {

                // cout << "word: " << word << endl;
                // add all the column data
                // of a row to a vector
                row.push_back(word);
            }

            if (0 == strcasecmp(row[0].c_str(), "SWEEP"))
            {
                SWEEP data = {static_cast<uint32_t>(stoul(row[1])), static_cast<uint8_t>(stoul(row[2])), static_cast<uint8_t>(stoi(row[3])), static_cast<bool>(stoi(row[4]))};
                // cout << "it's sweep: ts: " << stoul(row[1]) << " hclk: " << stoi(row[4]) << endl;
                Poser::poser_Sweep(data);
            }

            if (0 == strcasecmp(row[0].c_str(), "SYNC"))
            {
                SYNC data = {static_cast<uint32_t>(stoul(row[1])), static_cast<uint8_t>(stoul(row[2])), static_cast<bool>(stoi(row[3])), static_cast<bool>(stoi(row[4]))};
                // cout << "it's sync" << endl;
                SpreePose spose = Poser::poser_Sync(data);

                cout << "returned pose: " << spose.Pos[0] << " " << spose.Pos[1] << " " << spose.Pos[2] << " "
                     << spose.Rot[0] << " " << spose.Rot[1] << " " << spose.Rot[2] << " " << spose.Rot[3] << " " << endl;
            }
        }
        fin.close();
    }

    else
        cout << "Unable to open file";

    return 0;
}