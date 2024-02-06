// core ROS2 functionality:
#include <rclcpp/rclcpp.hpp>
// Interface with the robot model and collision object:
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// Moveit Task Constructor:
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
// Pose Generation:
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mass_distribution_demo");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
        : node_{ std::make_shared<rclcpp::Node>("mass_distribution_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "cuboid";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { 0.05, 0.05, 0.06 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.3;
    pose.position.y = 0.0;
    pose.position.z = 1.06;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
    task_ = createTask();

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("mass distribution estimation task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "panda_arm";
    const auto& hand_group_name = "panda_gripper";
    const auto& hand_frame = "panda_hand";


    const std::map<std::string, double> grasp_object_config{
        {"panda_finger_joint1", 0.015},
        {"panda_finger_joint2", 0.015}
    };


    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    sampling_planner->setProperty("goal_position_tolerance", 0.005);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage* attach_object_stage =
            nullptr;  // Forward attach_object_stage to place pose generator

    // MTC Serial Container for picking the object:
    // Holds several subtasks
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              {"eef", "group", "ik_frame"});

        // Approaching the object by relative cartesian motion:
        {
            auto stage =
                    std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // Generator Stage to generate the grasp pose
        // A generator stage computes it's results without regard to the stages before and after it (User needs to create the correct stages before and after)
        {
            // Sample grasp pose
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject("cuboid");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);  // Hook into current state

            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.11; // Distance between flange and end-effector frame between the fingers


            // Compute IK
            auto wrapper =
                    std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));
        }

        // To be able to pick the object, a collision between hand and object needs to be allowed
        {
            auto stage =
                    std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("cuboid",
                                   task.getRobotModel()
                                           ->getJointModelGroup(hand_group_name)
                                           ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
            grasp->insert(std::move(stage));
        }

        // Allow collision between object and table for picking up
        {
            auto stage =
                    std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
            stage->allowCollisions("cuboid",
                                   "panda_table",
                                   true);
            grasp->insert(std::move(stage));
        }

        // Close the hand to pick the object:
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }

        // Attach object to the hand:
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("cuboid", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        // Lift the object with a moverelative stage:
        {
            auto stage =
                    std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }

    {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
                "move to place",
                mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                          { hand_group_name, sampling_planner } });
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
    }

    // Serial Container for placing the object:
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });

        {
            // Sample place pose
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("cuboid");

            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "cuboid";
            target_pose_msg.pose.position.y = 0.5;
            target_pose_msg.pose.orientation.w = 1.0;
            stage->setPose(target_pose_msg);
            stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

            // Compute IK
            auto wrapper =
                    std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("cuboid");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }

        {
            auto stage =
                    std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions("cuboid",
                                   task.getRobotModel()
                                           ->getJointModelGroup(hand_group_name)
                                           ->getLinkModelNamesWithCollisionGeometry(),
                                   false);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("cuboid", hand_frame);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "retreat");

            // Set retreat direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 0.5;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        task.add(std::move(place));
    }

    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("ready");
        task.add(std::move(stage));
    }

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}