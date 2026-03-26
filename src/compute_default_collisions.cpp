#include <rclcpp/rclcpp.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit_core/moveit/robot_model/robot_model.hpp>
#include <moveit_core/moveit/robot_state/robot_state.hpp>
#include <moveit_core/moveit/collision_detection/collision_common.hpp>

#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <random>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Simple argument parser
struct Args
{
    std::string urdf_path;
    unsigned int num_trials = 10000;
    unsigned int min_collision_fraction = 95;
};

Args parseArguments(int argc, char** argv)
{
    Args args;
    for(int i=1;i<argc;i++)
    {
        std::string key(argv[i]);
        if(key == "--urdf_path")
            args.urdf_path = argv[++i];
        else if(key == "--num_trials")
            args.num_trials = std::stoi(argv[++i]);
        else if(key == "--min_collision_fraction")
            args.min_collision_fraction = std::stoi(argv[++i]);
        else
            std::cerr << "Unknown argument: " << key << std::endl;
    }
    if(args.urdf_path.empty())
        throw std::runtime_error("--urdf_path is required");
    return args;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Args args = parseArguments(argc, argv);

    std::cout << "URDF: " << args.urdf_path << "\n";
    std::cout << "Trials: " << args.num_trials << "\n";
    std::cout << "Min collision fraction: " << args.min_collision_fraction << "\n";

    // Load URDF
    std::ifstream urdf_file(args.urdf_path);
    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                            std::istreambuf_iterator<char>());
    auto urdf_model = urdf::parseURDF(urdf_string);

    // Create RobotModel
    moveit::core::RobotModelPtr robot_model =
        std::make_shared<moveit::core::RobotModel>(urdf_model, nullptr);

    // Initialize SRDF
    srdf::Model srdf_model;
    srdf_model.initString(*urdf_model, "");

    // Prepare collision tracking
    const auto& links = robot_model->getLinkModelNames();
    std::map<std::pair<std::string,std::string>, unsigned int> collision_counts;

    moveit::core::RobotState state(robot_model);
    std::random_device rd;
    std::mt19937 gen(rd());

    // Sampling loop
    for(unsigned int i=0;i<args.num_trials;i++)
    {
        state.setToRandomPositions();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        state.checkSelfCollision(req,res);

        for(const auto& contact : res.contacts)
        {
            auto link1 = contact.first.first;
            auto link2 = contact.first.second;
            if(link1 > link2) std::swap(link1,link2);
            collision_counts[{link1,link2}]++;
        }
    }

    // Categorize collisions
    json j;
    j["always"] = json::array();
    j["never"] = json::array();
    j["default"] = json::array();
    j["adjacent"] = json::array();

    for(size_t i=0;i<links.size();i++)
    {
        for(size_t j_idx=i+1;j_idx<links.size();j_idx++)
        {
            auto pair = std::make_pair(links[i], links[j_idx]);
            unsigned int count = collision_counts[pair];
            double fraction = 100.0 * count / args.num_trials;

            // Determine category
            bool adjacent = robot_model->getLinkModel(links[i])->getParentLinkModel() &&
                            robot_model->getLinkModel(links[i])->getParentLinkModel()->getName() == links[j_idx];

            if(adjacent)
                j["adjacent"].push_back({pair.first,pair.second});
            else if(fraction >= args.min_collision_fraction)
                j["always"].push_back({pair.first,pair.second});
            else if(fraction == 0)
                j["never"].push_back({pair.first,pair.second});
            else
                j["default"].push_back({pair.first,pair.second});

            // Add to SRDF as disabled collisions
            if(adjacent || fraction >= args.min_collision_fraction)
                srdf_model.disabled_collision_pairs_.emplace_back(pair.first,pair.second, adjacent ? "Adjacent" : "Always");
        }
    }

    // Write JSON
    std::ofstream json_file("collision_matrix.json");
    json_file << j.dump(4);
    json_file.close();
    std::cout << "JSON saved: collision_matrix.json\n";

    // Write SRDF
    std::ofstream srdf_file("robot.srdf");
    srdf_model.writeSRDF(srdf_file);
    srdf_file.close();
    std::cout << "SRDF saved: robot.srdf\n";

    rclcpp::shutdown();
    return 0;
}
