#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/collision.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <getopt.h>
#include <memory>
#include <vector>
#include <random>
#include <xbot2_interface/collision.h>

using json = nlohmann::json;

struct Options
{
    std::string urdf_path;
    int num_trials = 1000;
    double min_collision_fraction = 0.1;
};

Options parseArguments(int argc, char** argv)
{
    Options opts;

    static struct option long_options[] =
        {
            {"urdf_path", required_argument, 0, 'u'},
            {"num_trials", required_argument, 0, 'n'},
            {"min_collision_fraction", required_argument, 0, 'm'},
            {0,0,0,0}
        };

    int c;
    while((c = getopt_long(argc, argv, "u:n:m:", long_options, nullptr)) != -1)
    {
        switch(c)
        {
        case 'u': opts.urdf_path = optarg; break;
        case 'n': opts.num_trials = std::stoi(optarg); break;
        case 'm': opts.min_collision_fraction = std::stod(optarg); break;
        }
    }

    if(opts.urdf_path.empty())
        throw std::runtime_error("Missing --urdf_path");

    return opts;
}

std::string ReadFile(std::string path)
{
    std::ifstream t(path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}


int main(int argc, char** argv)
{
    Options opts = parseArguments(argc, argv);

    auto urdf_string = ReadFile(opts.urdf_path);

    auto collision_model = XBot::ModelInterface::getModel(urdf_string, "pin");

    auto dist_calc = std::make_unique<XBot::Collision::CollisionModel>(collision_model);

    XBot::Collision::CollisionModel::LinkPairVector lpv = dist_calc->getCollisionPairs(false);
    Eigen::VectorXd distances;

    Eigen::VectorXd q = collision_model->getNeutralQ();

    for(int i=0; i<opts.num_trials; ++i)
    {
        if(collision_model->isFloatingBase())
            q.segment(7, q.size()-7) = collision_model->generateRandomQ().segment(7, q.size()-7);
        else
            q = collision_model->generateRandomQ();

        collision_model->setJointPosition(q);
        collision_model->update();

        dist_calc->update();

        distances.setZero(dist_calc->getNumCollisionPairs(false));
        dist_calc->computeDistance(distances, false, -1);

        lpv = dist_calc->getCollisionPairs(false);
    }


    return 0;
}
