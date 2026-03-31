#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/collision.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <getopt.h>
#include <set>
#include <nlohmann/json.hpp>

nlohmann::json j;

struct Options {
    std::string urdf_path;
    int num_trials = 1000;
    double max_collision_fraction = 0.95;
    double min_collision_fraction = 1e-3;
};

Options parseArguments(int argc, char** argv) {
    Options opts;
    static struct option long_options[] = {
        {"urdf_path", required_argument, 0, 'u'},
        {"num_trials", required_argument, 0, 'n'},
        {"min_collision_fraction", required_argument, 0, 'm'},
        {"max_collision_fraction", required_argument, 0, 'x'},
        {0,0,0,0}
    };

    int c;
    while((c = getopt_long(argc, argv, "u:n:m:x:", long_options, nullptr)) != -1) {
        switch(c) {
        case 'u': opts.urdf_path = optarg; break;
        case 'n': opts.num_trials = std::stoi(optarg); break;
        case 'm': opts.min_collision_fraction = std::stod(optarg); break;
        case 'x': opts.max_collision_fraction = std::stod(optarg); break;
        }
    }

    if(opts.urdf_path.empty())
        throw std::runtime_error("Missing --urdf_path");

    return opts;
}

std::string ReadFile(const std::string &path) {
    std::ifstream t(path);
    if(!t.is_open())
        throw std::runtime_error("Cannot open file: " + path);

    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

int main(int argc, char** argv) {

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    Options opts = parseArguments(argc, argv);

    auto urdf_string = ReadFile(opts.urdf_path);
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(urdf_string, "pin");
    auto coll_model = std::make_unique<XBot::Collision::CollisionModel>(model);

    Eigen::VectorXd q = model->getNeutralQ();

    auto pairs_raw = coll_model->getCollisionPairs(false);

    for(const auto & pair : pairs_raw)
        std::cout << pair.first << " <---> " << pair.second << std::endl;

    std::map<std::pair<std::string,std::string>, std::vector<double>> map_pair_distances;

    /* Preallocate map entries and vectors */
    for(const auto &pair : pairs_raw)
    {
        auto &vec = map_pair_distances[pair];
        vec.reserve(opts.num_trials);
    }

    Eigen::VectorXd distances(pairs_raw.size());

    for(int i=0; i<opts.num_trials; ++i)
    {
        if(model->isFloatingBase())
            q.segment(7,q.size()-7) = model->generateRandomQ().segment(7,q.size()-7);
        else
            q = model->generateRandomQ();

        model->setJointPosition(q);
        model->update();
        coll_model->update();

        coll_model->computeDistance(distances, false, -1);

        for(size_t j=0; j<pairs_raw.size(); ++j)
        {
            map_pair_distances[pairs_raw[j]].push_back(distances[j]);
        }

        double progress = 100.0*(i+1)/opts.num_trials;
        std::cout << "\rProgress: " << progress << "% " << std::flush;
    }

    std::cout << std::endl;

    std::vector<std::pair<std::string,std::string>> never_in_collision;
    std::vector<std::pair<std::string,std::string>> always_in_collision;

    std::set<std::pair<std::string,std::string>> never_set;
    std::set<std::pair<std::string,std::string>> always_set;

    double collision_threshold = 1e-3;

    for(const auto &kv : map_pair_distances)
    {
        const auto &pair = kv.first;
        const auto &distances_vec = kv.second;

        int num_in_collision = 0;

        for(double d : distances_vec)
            if(d <= collision_threshold)
                num_in_collision++;

        double fraction = static_cast<double>(num_in_collision) / distances_vec.size();

        std::cout << pair.first << " <-> " << pair.second
                  << "  collision rate: " << fraction << std::endl;

        if(fraction <= opts.min_collision_fraction)
        {
            never_in_collision.push_back(pair);
            never_set.insert(pair);
        }
        else if(fraction >= opts.max_collision_fraction)
        {
            always_in_collision.push_back(pair);
            always_set.insert(pair);
        }
    }

    std::cout << "Never in collision (" << never_in_collision.size() << "):\n";
    for(auto &s : never_in_collision)
        std::cout << "  " << s.first <<" || "<<s.second<<std::endl;

    std::cout << "Always in collision (" << always_in_collision.size() << "):\n";
    for(auto &s : always_in_collision)
        std::cout << "  " << s.first <<" X "<<s.second<<std::endl;

    std::vector<std::pair<std::string,std::string>> collision_pairs;

    for(const auto &pair : pairs_raw)
    {
        if(!never_set.count(pair) && !always_set.count(pair))
            collision_pairs.push_back(pair);
    }

    std::cout << "Default collision pairs (" << collision_pairs.size() << "):\n";
    for(auto &s : collision_pairs)
        std::cout << "  " << s.first <<" <-> "<<s.second<<std::endl;

    j["collision_list"] = nlohmann::json::array();

    for(const auto& p : collision_pairs)
    {
        j["collision_list"].push_back({p.first, p.second});
    }

    std::ofstream file("collision_pairs.json");
    file << j.dump(4);   // pretty print with indentation
    file.close();

    std::cout << "Saved collision_pairs.json" << std::endl;

    return 0;
}