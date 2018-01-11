
#include "../revisit-logger/cpp/logger.hpp"

#include <dart/dart.hpp>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Eigen;

#include <dart/collision/bullet/bullet.hpp>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

using std::string;
using std::vector;


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
    unsigned num_wegs;
};

struct PDController
{
    double kp_, kd_, dt_;
    PDController(double kp, double kd, double dt) : kp_(kp), kd_(kd), dt_(dt) {}
    double get_output(double r, double y, double dy) {
        return kp_ * (r - y) + kd_ * dy;
    }
};


// Centimers conversion from SI units
constexpr long double operator"" _cm (long double meters) {
    return meters * 0.01;
}
constexpr long double operator"" _cm (unsigned long long meters) {
    return meters * 0.01;
}


// Notation for SI density
constexpr long double operator"" _kg_per_m3 (long double density) {
    return density;
}
constexpr long double operator"" _kg_per_m3 (unsigned long long density) {
    return density;
}


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
    auto body_joint_pair = skel->createJointAndBodyNodePair<FreeJoint>(
        nullptr, joint_prop, body_prop);

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
    weg_joint_prop.mName = wp.name + "_weg_joint";
    Isometry3d tf_weg{Isometry3d::Identity()};
    tf_weg.translation() = Vector3d(0, wp.dimensions.y() / 1.9, 0);
    weg_joint_prop.mT_ParentBodyToJoint = tf_weg;
    BodyNode::Properties weg_body_prop;
    weg_body_prop.mName = wp.name + "_weg";
    weg_body_prop.mRestitutionCoeff = wp.restitution;
    weg_body_prop.mInertia.setMass(weg_mass);
    weg_body_prop.mInertia.setMoment(weg_shape->computeInertia(weg_mass));
    auto weg_pair = skel->createJointAndBodyNodePair<PrismaticJoint>(body_joint_pair.second, weg_joint_prop, weg_body_prop);
    weg_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(weg_shape);
    weg_pair.first->setActuatorType(Joint::VELOCITY);
    weg_pair.first->setAxis(Vector3d{0, 1, 0});


    return body_joint_pair;
}


auto add_obstacle()
{
    auto skel = Skeleton::create();

    // auto dims = dart::math::randomVector<3>(8_cm, 15_cm);
    Vector3d dims(10_cm, 6_cm, 40_cm);

    ShapePtr shape{new BoxShape(dims)};
    const double mass = dart::math::random(0.5, 1.5) * 1000 * shape->getVolume();

    // Setup the joint properties
    WeldJoint::Properties joint_prop;

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = Vector3d(0.5, 0, 0);
    joint_prop.mT_ParentBodyToJoint = tf;

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = "obstacle1";
    body_prop.mRestitutionCoeff = 0.8;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = skel->createJointAndBodyNodePair<WeldJoint>(
        nullptr, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    return std::make_pair(skel, dims);
}


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


int main()
{
    // General parameters
    constexpr double material_density = 700_kg_per_m3;
    constexpr double material_restitution = 0.75;
    constexpr double vertical_offset = 20_cm;

    // Chassis parameters
    constexpr double wheel_base = 10_cm;
    constexpr double track_width = 12_cm;
    constexpr double chassis_height = 4_cm;
    const string chassis_name{"chassis"};
    const Vector3d chassis_dimensions{wheel_base, chassis_height, track_width};

    // Wheel parameters
    constexpr double wheel_radius = 2.5_cm;
    constexpr double wheel_thickness = 1.5_cm;
    const Vector3d wheel_dimensions{wheel_radius * 2, wheel_radius * 2, wheel_thickness};

    // Weg parameters
    constexpr double weg_radius = 0.25_cm;

    //
    // Create the UGV skeleton
    //

    auto ugv = Skeleton::create("ugv");


    //
    // Create the chassis as part of the UGV and attach it to the world
    //

    FreeJoint* chassis_joint;
    BodyNode* chassis_body;
    std::tie(chassis_joint, chassis_body) = add_chassis(
        ugv, chassis_name, chassis_dimensions, material_density, material_restitution);


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
        1
    };

    vector<string> wheel_names{
        "front-right-wheel", "front-left-wheel", "back-right-wheel", "back-left-wheel"
    };
    vector<long> wheel_idxs;
    vector<long> weg_idxs;

    for (const auto & name : wheel_names) {
        wheel_props.name = name;
        add_wheel(ugv, wheel_props);
        wheel_idxs.push_back(ugv->getDof(name + "_joint")->getIndexInSkeleton());
        weg_idxs.push_back(ugv->getDof(name + "_weg_joint")->getIndexInSkeleton());
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

    // The Bullet collision detector uses primitives instead of meshes, which makes
    // it faster and more useful for this simple application.
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
        world->getConstraintSolver()->setCollisionDetector(
            dart::collision::CollisionDetector::getFactory()->create("bullet"));
    } else {
        cerr << "NO BULLET" << endl;
    }

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

    auto obstacle1 = add_obstacle();
    world->addSkeleton(obstacle1.first);


    //
    // Simulate the world for some amount of time
    //

    constexpr double TIME_STOP = 10;
    constexpr double TIME_STEP = 0.005;
    constexpr double VIS_STEP = 1.0 / 30.0;
    constexpr double VIS_SCALE = 500;
    world->setTimeStep(TIME_STEP);

    // Create the Revisit logger
    revisit::logger rl(0.0, VIS_STEP, TIME_STOP);

    rl.add_box(chassis_name,
        chassis_dimensions.x() * VIS_SCALE,
        chassis_dimensions.y() * VIS_SCALE,
        chassis_dimensions.z() * VIS_SCALE);

    for (const auto & name : wheel_names) {
        rl.add_ellipsoid(name,
            wheel_dimensions.x() * VIS_SCALE,
            wheel_dimensions.y() * VIS_SCALE,
            wheel_dimensions.z() * VIS_SCALE);

        rl.add_sphere(name + "_weg", weg_radius * VIS_SCALE);
    }

    rl.add_box("obstacle1",
        obstacle1.second.x() * VIS_SCALE,
        obstacle1.second.y() * VIS_SCALE,
        obstacle1.second.z() * VIS_SCALE);

    //
    // Controllers
    //

    PDController weg_pd{1, 0.1, TIME_STEP};


    double next_vis_output_time = 0;
    int count = 0;
    while (world->getTime() < TIME_STOP + TIME_STEP/2.0) {

        world->step();

        auto wheel_speed = -10;
        ugv->setCommand(wheel_idxs.at(0), wheel_speed);
        ugv->setCommand(wheel_idxs.at(1), wheel_speed);
        ugv->setCommand(wheel_idxs.at(2), wheel_speed);
        ugv->setCommand(wheel_idxs.at(3), wheel_speed);


        auto r = 1.5_cm;
        auto y = ugv->getJoint("front-right-wheel_weg_joint")->getPosition(0);
        auto dy = ugv->getJoint("front-right-wheel_weg_joint")->getVelocity(0);
        auto u = weg_pd.get_output(r, y, dy);

        if (++count >= 10) {
            // cout << world->getTime() << " " << r << " " << y << " " << dy << " " << u << endl;
            count = 0;
        }

        ugv->setCommand(weg_idxs.at(0), u);
        ugv->setCommand(weg_idxs.at(1), u);
        ugv->setCommand(weg_idxs.at(2), u);
        ugv->setCommand(weg_idxs.at(3), u);


        if (world->getTime() > next_vis_output_time) {

            // auto chassis_T = chassis_body->getTransform();
            // auto ch_t = chassis_T.translation() * VIS_SCALE;
            // Quaterniond ch_q(chassis_T.rotation());

            // rl.add_frame(chassis_name,
            //     ch_t.x(), ch_t.y(), ch_t.z(),
            //     ch_q.x(), ch_q.y(), ch_q.z(), ch_q.w());

            // for (const auto & name : wheel_names) {
            //     auto wheel_T = ugv->getBodyNode(name)->getTransform();
            //     auto wh_t = wheel_T.translation() * VIS_SCALE;
            //     Quaterniond wh_q(wheel_T.rotation());

            //     rl.add_to_frame(name,
            //         wh_t.x(), wh_t.y(), wh_t.z(),
            //         wh_q.x(), wh_q.y(), wh_q.z(), wh_q.w());

            //     auto weg_T = ugv->getBodyNode(name + "_weg")->getTransform();
            //     auto wg_t = weg_T.translation() * VIS_SCALE;
            //     Quaterniond wg_q(weg_T.rotation());

            //     rl.add_to_frame(name + "_weg",
            //         wg_t.x(), wg_t.y(), wg_t.z(),
            //         wg_q.x(), wg_q.y(), wg_q.z(), wg_q.w());
            // }

            add_frame_to_rl(rl, world, VIS_SCALE);

            next_vis_output_time += VIS_STEP;
        }

    }

    // Passing false prints a compact JSON representation
    cout << rl.to_string() << endl;
    return EXIT_SUCCESS;
}
