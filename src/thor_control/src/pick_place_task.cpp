#include <thor_control/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <actionlib/client/simple_action_client.h>
#include <shape_msgs/Mesh.h>
#include <ros/package.h>
#include <thor_control/mesh_loader.h>


namespace thor_control {

    constexpr char LOGNAME[] = "pick_place_task";
    constexpr char PickPlaceTask::LOGNAME[];

    PickPlaceTask::PickPlaceTask(const std::string &task_name, const ros::NodeHandle &pnh)
        : pnh_(pnh), task_name_(task_name) {

        loadParameters();
        ros::Duration(1.0).sleep(); // Wait for ApplyPlanningScene service
        moveit::planning_interface::PlanningSceneInterface psi;
        object_ = createObject();
        spawnObject(psi);
    }

    moveit_msgs::CollisionObject PickPlaceTask::createObject() {
        // Load the mesh using the mesh_loader function
        shape_msgs::Mesh mesh = loadMeshRviz(object_name_, object_scale_);
        object_bbox_ = getBoundingBox(mesh);

        moveit_msgs::CollisionObject object;
        object.id = object_name_;
        object.header.frame_id = object_reference_frame_;

        object.meshes.resize(1);
        object.meshes[0] = mesh;
        object.mesh_poses.push_back(object_pose_);

        loadMeshGazebo(object_name_, object_scale_, object_pose_);

        return object;
    }

    void PickPlaceTask::spawnObject(moveit::planning_interface::PlanningSceneInterface &psi) {
        if (!psi.applyCollisionObject(object_))
            throw std::runtime_error("Failed to spawn object: " + object_.id);
    }

    void PickPlaceTask::loadParameters() {
        /****************************************************
         *                                                  *
         *               Load Parameters                    *
         *                                                  *
         ***************************************************/
        ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

        // Planning group properties
        size_t errors = 0;
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", arm_group_name_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", hand_group_name_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", eef_name_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", hand_frame_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "grasp_frame_transform", grasp_frame_transform_);

        // Predefined pose targets
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", hand_open_pose_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", hand_close_pose_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);

        // Target object
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_name", object_name_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_reference_frame", object_reference_frame_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link1", surface_link1_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link2", surface_link2_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_scale", object_scale_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_pose", object_pose_);
        support_surfaces_ = {surface_link1_, surface_link2_};

        // Pick/Place metrics
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", approach_object_min_dist_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", approach_object_max_dist_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
        errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", place_pose_);
        rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
    }

    bool PickPlaceTask::init() {
        ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");

        // Reset ROS introspection before constructing the new object
        // TODO(v4hn): global storage for Introspection services to enable one-liner
        task_.reset();
        task_.reset(new moveit::task_constructor::Task());

        Task &t = *task_;
        t.stages()->setName(task_name_);
        t.loadRobotModel();

        // Sampling planner
        auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

        // Cartesian planner
        auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.001);

        // Set task properties
        t.setProperty("group", arm_group_name_);
        t.setProperty("eef", eef_name_);
        t.setProperty("hand", hand_group_name_);
        t.setProperty("hand_grasping_frame", hand_frame_);
        t.setProperty("ik_frame", hand_frame_);

        /****************************************************
         *                                                  *
         *               Current State                      *
         *                                                  *
         ***************************************************/
        {
            auto current_state = std::make_unique<stages::CurrentState>("current state");

            // Verify that object is not attached
            auto applicability_filter =
                std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
            applicability_filter->setPredicate([this](const SolutionBase &s, std::string &comment)
                                               {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object_name_)) {
				comment = "object with id '" + object_name_ + "' is already attached and cannot be picked";
				return false;
			}
			return true; });
            t.add(std::move(applicability_filter));
        }

        /****************************************************
         *                                                  *
         *               Open Hand                          *
         *                                                  *
         ***************************************************/
        Stage *initial_state_ptr = nullptr;
        { // Open Hand
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(hand_group_name_);
            stage->setGoal(hand_open_pose_);
            initial_state_ptr = stage.get(); // remember start state for monitoring grasp pose generator
            t.add(std::move(stage));
        }

        /****************************************************
         *                                                  *
         *               Move to Pick                       *
         *                                                  *
         ***************************************************/
        { // Move-to pre-grasp
            auto stage = std::make_unique<stages::Connect>(
                "move to pick", stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            t.add(std::move(stage));
        }

        /****************************************************
         *                                                  *
         *               Pick Object                        *
         *                                                  *
         ***************************************************/
        Stage *pick_stage_ptr = nullptr;
        {
            auto grasp = std::make_unique<SerialContainer>("pick object");
            t.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
            grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            /****************************************************
      ---- *               Approach Object                    *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
                stage->properties().set("marker_ns", "approach_object");
                stage->properties().set("link", hand_frame_);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

                // Set hand forward direction
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = hand_frame_;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
                
            }

            /****************************************************
      ---- *               Generate Grasp Pose                *
             ***************************************************/
            {
                // Sample grasp pose
                auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(hand_open_pose_);
                stage->setObject(object_name_);
                stage->setAngleDelta(M_PI / 12);
                stage->setMonitoredStage(initial_state_ptr); // hook into successful initial-phase solutions

                // Compute IK
                auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(16);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
                wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
                grasp->insert(std::move(wrapper));
            }

            /****************************************************
      ---- *               Allow Collision (hand object)   *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
                stage->allowCollisions(
                    object_name_, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
                    true);
                grasp->insert(std::move(stage));
            }

            /****************************************************
      ---- *               Close Hand                      *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
                stage->setGroup(hand_group_name_);
                stage->setGoal(hand_close_pose_);
                grasp->insert(std::move(stage));
            }

            /****************************************************
      .... *               Attach Object                      *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
                stage->attachObject(object_name_, hand_frame_);
                grasp->insert(std::move(stage));
            }

            /****************************************************
      .... *               Allow collision (object support)   *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
                stage->allowCollisions({object_name_}, support_surfaces_, true);
                grasp->insert(std::move(stage));
            }

            /****************************************************
      .... *               Lift object                        *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
                stage->setIKFrame(hand_frame_);
                stage->properties().set("marker_ns", "lift_object");

                // Set upward direction
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = world_frame_;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }

            /****************************************************
      .... *               Forbid collision (object support)  *
             ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
                stage->allowCollisions({object_name_}, support_surfaces_, false);
                grasp->insert(std::move(stage));
            }

            pick_stage_ptr = grasp.get(); // remember for monitoring place pose generator

            // Add grasp container to task
            t.add(std::move(grasp));
        }

        /******************************************************
         *                                                    *
         *          Move to Place                             *
         *                                                    *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::Connect>(
                "move to place", stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            t.add(std::move(stage));
        }

        /******************************************************
         *                                                    *
         *          Place Object                              *
         *                                                    *
         *****************************************************/
        {
            auto place = std::make_unique<SerialContainer>("place object");
            t.properties().exposeTo(place->properties(), {"eef", "hand", "group"});
            place->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group"});

            /******************************************************
      ---- *          Lower Object                              *
             *****************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
                stage->properties().set("marker_ns", "lower_object");
                stage->properties().set("link", hand_frame_);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(.03, .13);

                // Set downward direction
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = world_frame_;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                place->insert(std::move(stage));
            }

            /******************************************************
      ---- *          Generate Place Pose                       *
             *****************************************************/
            {
                // Generate Place Pose
                auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject(object_name_);

                // Set target pose
                geometry_msgs::PoseStamped p;
                p.header.frame_id = object_reference_frame_;
                p.pose = place_pose_;
                stage->setPose(p);
                stage->setMonitoredStage(pick_stage_ptr); // hook into successful pick solutions

                // Compute IK
                auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(2);
                wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
                wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
                place->insert(std::move(wrapper));
            }

            /******************************************************
      ---- *          Open Hand                              *
             *****************************************************/
            {
                auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
                stage->setGroup(hand_group_name_);
                stage->setGoal(hand_open_pose_);
                place->insert(std::move(stage));
            }

            /******************************************************
      ---- *          Forbid collision (hand, object)        *
             *****************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
                stage->allowCollisions(object_name_, *t.getRobotModel()->getJointModelGroup(hand_group_name_), false);
                place->insert(std::move(stage));
            }

            /******************************************************
      ---- *          Detach Object                             *
             *****************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
                stage->detachObject(object_name_, hand_frame_);
                place->insert(std::move(stage));
            }

            /******************************************************
      ---- *          Retreat Motion                            *
             *****************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0, .25);
                stage->setIKFrame(hand_frame_);
                stage->properties().set("marker_ns", "retreat");
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = hand_frame_;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                place->insert(std::move(stage));
            }

            // Add place container to task
            t.add(std::move(place));
        }

        /******************************************************
         *                                                    *
         *          Move to Home                              *
         *                                                    *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
            stage->properties().configureInitFrom(Stage::PARENT, {"group"});
            stage->setGoal(arm_home_pose_);
            stage->restrictDirection(stages::MoveTo::FORWARD);
            t.add(std::move(stage));
        }

        // prepare Task structure for planning
        try {
            t.init();
        }
        catch (InitStageException &e) {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
            return false;
        }

        return true;
    }

    bool PickPlaceTask::plan() {
        ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
        int max_solutions = pnh_.param<int>("max_solutions", 10);

        return static_cast<bool>(task_->plan(max_solutions));
    }

    bool PickPlaceTask::execute() {
        ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
        moveit_msgs::MoveItErrorCodes execute_result;

        // execute_result = task_->execute(*task_->solutions().front());
        // // If you want to inspect the goal message, use this instead:
        actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
        execute("execute_task_solution", true); execute.waitForServer();
        moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
        task_->solutions().front()->toMsg(execute_goal.solution);
        execute.sendGoalAndWait(execute_goal);
        execute_result = execute.getResult()->error_code;

        if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
            return false;
        }

        return true;
    }
}