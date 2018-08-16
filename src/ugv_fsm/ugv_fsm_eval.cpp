

#ifdef VISUALIZE
    #include "../../logger/cpp/logger.hpp"
#endif

#include "../extras/pd_controller.hpp"
#include "../extras/utilities.hpp"


#include <dart/dart.hpp>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math::suffixes;
using namespace Eigen;

#include <dart/collision/bullet/bullet.hpp>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

using std::string;
using std::vector;

using std::min;
using std::max;


struct WheelProperties
{
    Vector3d dimensions;
    double restitution;
    double density;
    double abs_x_offset;
    double abs_z_offset;
    BodyNode* parent;
    string name;

    double weg_radius;
    size_t num_wegs;
};



auto add_chassis(SkeletonPtr skel, const string & name, const Vector3d & dims,
    double density, double restitution)
{
    ShapePtr shape{new BoxShape(dims)};
    const double mass = density * shape->getVolume();

    // Setup the joint properties
    FreeJoint::Properties joint_prop;
    joint_prop.mName = name + "_joint";

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = restitution;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = skel->createJointAndBodyNodePair<FreeJoint>(nullptr,
        joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    return body_joint_pair;
}


auto add_wheel(SkeletonPtr skel, const WheelProperties & wp)
{
    ShapePtr shape{new EllipsoidShape{wp.dimensions}};
    const double mass = wp.density * shape->getVolume();

    // Setup the wheel properties
    RevoluteJoint::Properties joint_prop;
    joint_prop.mName = wp.name + "_joint";

    auto x_offset = wp.abs_x_offset * (wp.name.find("front") != string::npos ? 1 : -1);
    auto z_offset = wp.abs_z_offset * (wp.name.find("right") != string::npos ? 1 : -1);

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(x_offset, 0, z_offset);
    joint_prop.mT_ParentBodyToJoint = tf;

    BodyNode::Properties body_prop;
    body_prop.mName = wp.name;
    body_prop.mRestitutionCoeff = wp.restitution;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair =
        skel->createJointAndBodyNodePair<RevoluteJoint>(wp.parent, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    // Set the actuator type of the wheel joint
    body_joint_pair.first->setActuatorType(Joint::VELOCITY);

    //
    // Wegs
    //
    ShapePtr weg_shape{new SphereShape{wp.weg_radius}};
    const double weg_mass = wp.density * weg_shape->getVolume();
    PrismaticJoint::Properties weg_joint_prop;
    BodyNode::Properties weg_body_prop;
    weg_body_prop.mRestitutionCoeff = wp.restitution;
    weg_body_prop.mInertia.setMass(weg_mass);
    weg_body_prop.mInertia.setMoment(weg_shape->computeInertia(weg_mass));

    for (size_t weg_idx = 0; weg_idx < wp.num_wegs; ++weg_idx) {

        weg_body_prop.mName = wp.name + "_weg" + std::to_string(weg_idx);
        weg_joint_prop.mName = wp.name + "_weg_joint" + std::to_string(weg_idx);

        // Move the weg into place
        Isometry3d tf_weg{Isometry3d::Identity()};
        auto vector_to_point = Vector3d(0, wp.dimensions.y() / 2.0 - wp.weg_radius, 0);
        auto vector_transform = AngleAxisd(2_pi * weg_idx / wp.num_wegs, Vector3d(0, 0, 1));
        tf_weg.translation() = vector_transform * vector_to_point;

        weg_joint_prop.mT_ParentBodyToJoint = tf_weg;

        auto weg_pair = skel->createJointAndBodyNodePair<PrismaticJoint>(body_joint_pair.second,
            weg_joint_prop, weg_body_prop);
        weg_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(weg_shape);
        weg_pair.first->setActuatorType(Joint::VELOCITY);

        weg_pair.first->setAxis(vector_transform * vector_to_point);
    }


    return body_joint_pair;
}


auto create_random_box(double ugv_density, size_t rseed)
{
    static std::mt19937 reng(rseed);

    // Randomize the size of objects
    static std::uniform_real_distribution<double> uni_real_xz_size(5_cm, 20_cm);
    static std::uniform_real_distribution<double> uni_real_y_size(2_cm, 5_cm);

    // Density factor
    static std::uniform_real_distribution<double> uni_real_density(0.5, 1.5);

    // Make sure that obstacles don't fall on the robot
    static const std::array<double, 4> intervals{{-250_cm, -24_cm, 24_cm, 250_cm}};
    static const std::array<double, 3> weights{{1, 0, 1}};
    static std::piecewise_constant_distribution<double> uni_real_xz_trans(
        intervals.begin(), intervals.end(), weights.begin());

    static size_t object_count = 0;

    std::string name = "obstacle" + std::to_string(object_count++);

    auto skel = Skeleton::create(name);

    Vector3d dims, trans;

    dims.x() = uni_real_xz_size(reng);
    dims.y() = uni_real_y_size(reng);
    dims.z() = uni_real_xz_size(reng);

    trans.x() = uni_real_xz_trans(reng);
    trans.y() = 10_cm;
    trans.z() = uni_real_xz_trans(reng);

    ShapePtr shape{new BoxShape(dims)};
    const double mass = uni_real_density(reng) * ugv_density * shape->getVolume();

    // Setup the joint properties
    FreeJoint::Properties joint_prop;

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = trans;
    joint_prop.mT_ParentBodyToJoint = tf;

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = name;
    body_prop.mRestitutionCoeff = 0.8;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = skel->createJointAndBodyNodePair<FreeJoint>(
        nullptr, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    return std::make_tuple(skel, dims, name);
}


#ifdef VISUALIZE
    void add_frame_to_rl(revisit::logger & logger, WorldPtr & world, double scale) {

        logger.new_frame();

        for (size_t skel_idx = 0; skel_idx < world->getNumSkeletons(); ++skel_idx) {

            auto skel = world->getSkeleton(skel_idx);
            for (const auto & bnode : skel->getBodyNodes()) {

                auto T = bnode->getTransform();
                auto trans = T.translation() * scale;
                Quaterniond quat(T.rotation());

                logger.add_to_frame(
                    bnode->getName(),
                    trans.x(), trans.y(), trans.z(),
                    quat.x(), quat.y(), quat.z(), quat.w());
            }
        }
    }
#endif


int main(int argc, char const *argv[])
{
    // General parameters
    constexpr double material_density = 700_kg_per_m3;
    constexpr double material_restitution = 0.75;
    constexpr double vertical_offset = 5_cm;


    //
    // Parse program arguments
    //


    if (argc < 2) {
        cerr << "Not enough program arguments." << endl;
        return 1;
    }

    std::istringstream iss(argv[1]);

    // Simulation parameters
    double TIME_STOP;
    iss >> TIME_STOP;

#ifdef VISUALIZE
    cerr << "TIME_STOP " << TIME_STOP;
#endif

    constexpr double TIME_STEP = 0.005;

    // Number of obstacles
    size_t num_obstacles, obstacle_seed;
    iss >> num_obstacles >> obstacle_seed;

#ifdef VISUALIZE
    cerr << "\nnum_obstacles " << num_obstacles << "\nobstacle_seed " << obstacle_seed;
#endif

    // Chassis parameters
    double wheel_base, track_width;
    iss >> wheel_base >> track_width;

#ifdef VISUALIZE
    cerr << "\nwheel_base " << wheel_base  << "\ntrack_width " << track_width;
#endif

    constexpr double chassis_height = 3_cm;
    const string chassis_name{"chassis"};
    const Vector3d chassis_dimensions{wheel_base, chassis_height, track_width};

    // Wheel parameters
    double wheel_radius;
    iss >> wheel_radius;

#ifdef VISUALIZE
    cerr << "\nwheel_radius " << wheel_radius;
#endif

    constexpr double wheel_thickness = 1.5_cm;
    const Vector3d wheel_dimensions{wheel_radius * 2, wheel_radius * 2, wheel_thickness};

    // Weg parameters
    size_t weg_count;
    double weg_extension_slope, weg_extension_intercept;
    iss >> weg_count >> weg_extension_slope >> weg_extension_intercept;

#ifdef VISUALIZE
    cerr << "\nweg_count " << weg_count
         << "\nweg_extension_slope " << weg_extension_slope
         << "\nweg_extension_intercept " << weg_extension_intercept;
#endif

    constexpr double weg_radius = 0.25_cm;

    // FSM parameters
    double forward_speed_scale;
    double forward_to_left_lo;//, forward_to_left_hi;
    double /*forward_to_right_lo,*/ forward_to_right_hi;

    iss >> forward_speed_scale
        // >> forward_right
        >> forward_to_left_lo
        // >> forward_to_left_hi
        // >> forward_to_right_lo
        >> forward_to_right_hi;

#ifdef VISUALIZE
    cerr << "\nforward_speed_scale " << forward_speed_scale
         // << "\nforward_right " << forward_right
         << "\nforward_to_left_lo " << forward_to_left_lo * 180 / 3.145926
    //      << "\nforward_to_left_hi " << forward_to_left_hi
    //      << "\nforward_to_right_lo " << forward_to_right_lo
         << "\nforward_to_right_hi " << forward_to_right_hi * 180 / 3.145926;
#endif

    double left_left_scale, left_right_scale;
    double /*left_to_forward_lo,*/ left_to_forward_hi;

    iss >> left_left_scale
        >> left_right_scale
        // >> left_to_forward_lo
        >> left_to_forward_hi;

#ifdef VISUALIZE
    cerr << "\nleft_left " << left_left_scale
         << "\nleft_right " << left_right_scale
    //      << "\nleft_to_forward_lo " << left_to_forward_lo
         << "\nleft_to_forward_hi " << left_to_forward_hi * 180 / 3.145926;
#endif

    double right_left_scale, right_right_scale;
    double right_to_forward_lo;//, right_to_forward_hi;

    iss >> right_left_scale
        >> right_right_scale
        >> right_to_forward_lo;
        // >> right_to_forward_hi;

#ifdef VISUALIZE
    cerr << "\nright_left " << right_left_scale
         << "\nright_right " << right_right_scale
         << "\nright_to_forward_lo " << right_to_forward_lo * 180 / 3.145926 << endl;
    //      << "\nright_to_forward_hi " << right_to_forward_hi << endl;
#endif

#ifdef VISUALIZE

    constexpr double VIS_STEP = 1.0 / 10.0;
    constexpr double VIS_SCALE = 10;


    //
    // Create the visualization logger
    //

    revisit::logger rl(VIS_STEP, TIME_STOP);
#endif


    //
    // Create the UGV skeleton
    //

    auto ugv = Skeleton::create("ugv");


    //
    // Create the chassis as part of the UGV and attach it to the world
    //

    FreeJoint* chassis_joint;
    BodyNode* chassis_body;
    std::tie(chassis_joint, chassis_body) = add_chassis(ugv, chassis_name,
        chassis_dimensions, material_density, material_restitution);


#ifdef VISUALIZE
    rl.add_box(chassis_name,
        chassis_dimensions.x() * VIS_SCALE,
        chassis_dimensions.y() * VIS_SCALE,
        chassis_dimensions.z() * VIS_SCALE);
#endif


    //
    // Create the wheels and attach them to the chassis
    //

    WheelProperties wheel_props{
        wheel_dimensions,
        material_restitution,
        material_density,
        chassis_dimensions.x() / 2.0,
        chassis_dimensions.z() / 2.0,
        chassis_body,
        "",
        weg_radius,
        weg_count
    };

    vector<string> wheel_names{
        "front-right-wheel", "front-left-wheel", "back-right-wheel", "back-left-wheel"
    };
    vector<string> weg_joint_names;
    vector<long> wheel_idxs;
    vector<long> weg_idxs;

    for (const auto & name : wheel_names) {
        wheel_props.name = name;
        add_wheel(ugv, wheel_props);
        wheel_idxs.push_back(ugv->getDof(name + "_joint")->getIndexInSkeleton());

#ifdef VISUALIZE
        rl.add_ellipsoid(name,
            wheel_dimensions.x() * VIS_SCALE,
            wheel_dimensions.y() * VIS_SCALE,
            wheel_dimensions.z() * VIS_SCALE);
#endif

        for (size_t weg_idx = 0; weg_idx < weg_count; ++weg_idx) {
            auto weg_name = name + "_weg_joint" + std::to_string(weg_idx);
            weg_idxs.push_back(ugv->getDof(weg_name)->getIndexInSkeleton());
            weg_joint_names.push_back(weg_name);

#ifdef VISUALIZE
            rl.add_sphere(name + "_weg" + std::to_string(weg_idx), weg_radius * VIS_SCALE);
#endif
        }
    }


    //
    // Create a ground plane
    //

    auto ground_depth = 1.0;

    auto ground = Skeleton::create("ground");
    auto ground_body = ground->createJointAndBodyNodePair<WeldJoint>().second;
    ground_body->setRestitutionCoeff(material_restitution);
    ground_body->setName("ground");
    ShapePtr ground_box(new BoxShape(Vector3d(100, ground_depth, 100)));
    ground_body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(ground_box);

    // Shift the ground so that its top is at y=0
    Isometry3d ground_tf(Isometry3d::Identity());
    ground_tf.translate(Vector3d(0.0, -ground_depth / 2.0, 0.0));
    ground_body->getParentJoint()->setTransformFromParentBodyNode(ground_tf);


    //
    // Create the world and add the ugv skeleton
    //

    WorldPtr world(new World);

    world->addSkeleton(ugv);
    world->addSkeleton(ground);
    world->setGravity(Vector3d(0.0, -9.80665, 0.0));

    // Position the UGV above ground
    Vector6d ugv_positions(Vector6d::Zero());
    ugv_positions[4] = vertical_offset;
    chassis_joint->setPositions(ugv_positions);


    //
    // Add obstacles
    //

    auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
    auto collisionGroup = world->getConstraintSolver()->getCollisionGroup();

    auto numObstaclesAdded = 0ul;

    for (size_t obs_idx = 0; obs_idx < num_obstacles; ++obs_idx) {
        SkeletonPtr skel;
        Vector3d dims;
        std::string name;

        std::tie(skel, dims, name) = create_random_box(material_density, obstacle_seed);

        // Only add object if there is not collision
        auto newGroup = collisionEngine->createCollisionGroup(skel.get());

        dart::collision::CollisionOption option;
        dart::collision::CollisionResult result;
        bool collision = collisionGroup->collide(newGroup.get(), option, &result);

        if(!collision) {
            world->addSkeleton(skel);

            ++numObstaclesAdded;

#ifdef VISUALIZE
            rl.add_box(name,
                dims.x() * VIS_SCALE,
                dims.y() * VIS_SCALE,
                dims.z() * VIS_SCALE);
#endif
        }
    }

#ifdef VISUALIZE
    cerr << "Number of obstacles : " << numObstaclesAdded << endl;
#endif

    // The Bullet collision detector uses primitives instead of meshes, which makes
    // it faster and more useful for this simple application.
    // if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
    //     world->getConstraintSolver()->setCollisionDetector(
    //         dart::collision::CollisionDetector::getFactory()->create("bullet"));
    // } else {
    //     cerr << "NO BULLET" << endl;
    // }


    //
    // Controllers
    //

    // 730 RPM --> 76.44542115 rad/s
    constexpr double MAX_ABS_RADS = 20;

    // The PD controller can be reused by all wegs
    PDController weg_pd{1, 0.1, TIME_STEP};

    // Direction controller
    struct State {
        std::string name;
        double left_speed, right_speed;
        double to_left_lo, to_left_hi;
        double to_right_lo, to_right_hi;
        double to_forward_lo, to_forward_hi;

        std::string transition(double angle) {
            if (to_left_lo < angle && angle < to_left_hi) {
                return "left";
            } else if (to_right_lo < angle && angle < to_right_hi) {
                return "right";
            } else if (to_forward_lo < angle && angle < to_forward_hi) {
                return "forward";
            } else {
                return name;
            }
        }
    };

    // double max_speed = 10;
    // std::unordered_map<std::string, State> fsm{{
    //     {"forward", {"forward", -max_speed, -max_speed, 10_deg, 2_pi, -2_pi, -10_deg,      1,    -1}},
    //     {"left",    {"left",     max_speed, -max_speed,       1,  -1,     1,      -1,  -2_pi, 5_deg}},
    //     {"right",   {"right",   -max_speed,  max_speed,       1,  -1,     1,      -1, -5_deg,  2_pi}},
    // }};

    std::unordered_map<std::string, State> fsm{{
        {"forward", {"forward",
            -MAX_ABS_RADS * forward_speed_scale,
            -MAX_ABS_RADS * forward_speed_scale,
            forward_to_left_lo, 2_pi,
            -2_pi, forward_to_right_hi,
            1, -1}
        },
        {"left",    {"left",
            -MAX_ABS_RADS * left_left_scale,
            -MAX_ABS_RADS * left_right_scale,
            1, -1,
            1, -1,
            -2_pi, left_to_forward_hi}
        },
        {"right",   {"right",
            -MAX_ABS_RADS * right_left_scale,
            -MAX_ABS_RADS * right_right_scale,
            1, -1,
            1, -1,
            right_to_forward_lo, 2_pi}
        }
    }};

    std::string state("forward");


    //
    // Simulate the world for some amount of time
    //

    world->setTimeStep(TIME_STEP);

    double next_control_update_time = 0;
    constexpr double CONTROL_STEP = 0.1;


#ifdef VISUALIZE
    double next_vis_output_time = 0;
    rl.add_sphere("target", 5_cm * VIS_SCALE);
#endif

    vector<Vector3d> targets{
        {-200_cm, 0,  200_cm},
        { 200_cm, 0,  200_cm},
        {-200_cm, 0, -200_cm},
        { 200_cm, 0, -200_cm},
    };

    size_t target_idx = 0;
    double target_dist = 0.0;
#ifdef VISUALIZE
    bool update_target = true;
#endif

    double left_speed = 0;
    double right_speed = 0;
    double weg_extension = 0;

    double angular_speed_error = 0;
    double linear_speed_error = 0;
    double alpha = 0.25;

#ifdef VISUALIZE
    cout << "time angle_scaled angular_speed_error_scaled linear_speed_error_scaled"
            " left_speed right_speed weg_extension target_idx x y z";
    cout << " state" << endl;
#endif

    while (world->getTime() < TIME_STOP + TIME_STEP/2.0) {

        world->step();

        if (world->getTime() >= next_control_update_time) {
            next_control_update_time += CONTROL_STEP;

            auto chassis_transform = ugv->getBodyNode(chassis_name)->getTransform();

            auto chassis_yaw = chassis_transform.rotation().eulerAngles(2, 0, 2).y();
            auto chassis_pos = chassis_transform.translation();

            auto local_to_global = AngleAxisd(chassis_yaw, Vector3d(0, 1, 0));

            auto Va = local_to_global * Vector3d(1, 0, 0);
            auto Vb = targets[target_idx] - chassis_pos;
            auto Vn = Vector3d(0, 1, 0);

            auto angle = std::atan2(Va.cross(Vb).dot(Vn), Va.dot(Vb));
            state = fsm[state].transition(angle);

            target_dist = (chassis_pos - targets[target_idx]).norm();

            if (target_dist < 8_cm) {
#ifdef VISUALIZE
                update_target = true;
#endif
                if (++target_idx >= targets.size()) {
                    break;
                }
            }

            double linear_speed = ugv->getBodyNode(chassis_name)->getLinearVelocity().norm();
            double angular_speed = ugv->getBodyNode(chassis_name)->getAngularVelocity().y();

            // expected angular speed: w = (Vr - Vl) / l
            double Vr = right_speed * wheel_radius;
            double Vl = left_speed * wheel_radius;

            double expected_angular_speed = -(Vr - Vl) / track_width;
            double expected_linear_speed = -(Vr + Vl) / 2.0;

            double angular_error = abs(angular_speed - expected_angular_speed);
            double linear_error = abs(linear_speed - expected_linear_speed);

            angular_speed_error = (alpha * angular_error) + (1.0 - alpha) * angular_speed_error;
            linear_speed_error = (alpha * linear_error) + (1.0 - alpha) * linear_speed_error;

            const double max_abs_linear_speed = MAX_ABS_RADS * wheel_radius;
            const double max_abs_angular_speed = 2 * max_abs_linear_speed / track_width;

            double angular_speed_error_scaled = angular_speed_error / max_abs_angular_speed;
            double linear_speed_error_scaled = linear_speed_error / max_abs_linear_speed;

            double w_angular = weg_extension_slope * abs(angular_speed_error_scaled) + weg_extension_intercept;
            double w_linear = weg_extension_slope * abs(linear_speed_error_scaled) + weg_extension_intercept;
            double w = max(
                min(1.0, max(0.0, w_angular)),
                min(1.0, max(0.0, w_linear)));

            const double max_w = wheel_radius - 1_cm;
            weg_extension = w * max_w;

            // Speed scaled by weg extension
            double w_speed_scale_factor = 1.0 - (w / 1.25);
            const double max_rads = MAX_ABS_RADS * w_speed_scale_factor;

            left_speed = max(-max_rads, min(max_rads, fsm[state].left_speed));
            right_speed = max(-max_rads, min(max_rads, fsm[state].right_speed));

#ifdef VISUALIZE
            double angle_scaled = (angle + 1_pi) / 2_pi;
            cout << world->getTime()
                 << " " << angle_scaled
                 << " " << angular_speed_error_scaled
                 << " " << linear_speed_error_scaled
                 << " " << left_speed
                 << " " << right_speed
                 << " " << weg_extension
                 << " " << target_idx
                 << " " << chassis_pos.x()
                 << " " << chassis_pos.y()
                 << " " << chassis_pos.z()
                 << " " << state
                 << endl;
#endif
        }

        ugv->setCommand(wheel_idxs.at(0), right_speed);
        ugv->setCommand(wheel_idxs.at(1), left_speed);
        ugv->setCommand(wheel_idxs.at(2), right_speed);
        ugv->setCommand(wheel_idxs.at(3), left_speed);

        vector<double> weg_commands;
        for (const auto & weg_joint_name : weg_joint_names) {
            auto y = ugv->getJoint(weg_joint_name)->getPosition(0);
            auto dy = ugv->getJoint(weg_joint_name)->getVelocity(0);
            auto u = weg_pd.get_output(weg_extension, y, dy);
            weg_commands.push_back(u);
        }

        for (size_t weg_idx = 0; weg_idx < weg_idxs.size(); ++weg_idx) {
            ugv->setCommand(weg_idxs.at(weg_idx), weg_commands.at(weg_idx));
        }


#ifdef VISUALIZE
        if (world->getTime() > next_vis_output_time) {
            add_frame_to_rl(rl, world, VIS_SCALE);
            if (update_target) {
                rl.add_to_frame("target",
                    targets[target_idx].x() * VIS_SCALE,
                    targets[target_idx].y() * VIS_SCALE,
                    targets[target_idx].z() * VIS_SCALE,
                    0, 0, 0, 1);
                update_target = false;
            }
            next_vis_output_time += VIS_STEP;
        }
#endif

    }

    // zero target_dist if all 4 targets have been reached
    target_dist = target_idx >= targets.size() ? 0 : target_dist;

#ifdef VISUALIZE
    rl.log_data_["duration"] = (rl.log_data_["frames"].size() - 1) * VIS_STEP;
    // Passing false prints a compact JSON representation
    cout << rl.to_string(false) << endl;

    cerr << "Targets reached : " << target_idx << endl;
    cerr << "Dist to next    : " << target_dist << endl;
    cerr << "Time remaining  : " << std::max(0.0, TIME_STOP - world->getTime()) << endl;
#else
    cout << target_idx
         << "," << target_dist
         << "," << std::max(0.0, TIME_STOP - world->getTime()) << endl;
#endif

    return EXIT_SUCCESS;
}
