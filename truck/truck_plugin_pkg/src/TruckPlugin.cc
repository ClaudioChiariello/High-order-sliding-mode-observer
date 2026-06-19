#include "TruckPlugin.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SpeedLimiter.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

using std::string;

enum FrontWheelType
{
    FL1 = 0,
    FR1 = 1,
    FL2 = 2,
    FR2 = 3
};


struct FrontWheel
{
    string driveJointName;
    string steerJointName;

    Entity driveJoint{kNullEntity};
    Entity steerJoint{kNullEntity};

    double driveSpeed{0};
    double steerSpeed{0};

    bool missingSteerJoint() const {return steerJoint == kNullEntity;}
    bool missingDriveJoint() const {return driveJoint == kNullEntity;}
};


struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), ang(0.0) {}
};




struct AxleWheelSpeeds
{
    double left;
    double right;
};


struct DerivedGeometry
{
    double dc;
    double Ld1;
    double Ld2; 
};


class gz::sim::systems::TruckPluginPrivate
{
    public:

        /// \brief Gazebo communication node.
        transport::Node node;

        /// \brief Model interface
        Model model{kNullEntity};

        /// \brief The model's canonical link.
        Link canonicalLink{kNullEntity};

        /// \brief Number of motorized wheels
        int drivenWheels{4};

        /// \brief Wheel radius
        double wheelRadius{0.5};

        /// \brief P gain for angular position.
        double gainPAng{1.0};

        /// \brief Maximum turning angle to limit steering to
        double steeringLimit{0.5};

        /// \brief Distance between 2 back wheel axes
        double rearWheelBase{1.0};

        /// \brief Distance between 2 front wheel axes
        double frontWheelBase{1.0};

        /// \brief Distance between 2 central wheel axes
        double centralWheelBase{1.0};

        /// \brief Distance between rear left and right wheels 
        double rearWheelSeparation{1.0};

        /// \brief Distance between front left and right wheels 
        double frontWheelSeparation{1.0};

        /// \brief Linear velocity limiter.
        std::unique_ptr<math::SpeedLimiter> limiterLin;

        /// \brief Angular velocity limiter.
        std::unique_ptr<math::SpeedLimiter> limiterAng;

        /// \brief Name of left wheel rotation joints
        std::vector<string> rearLeftJointNames;

        /// \brief Name of right wheel rotation joints
        std::vector<string> rearRightJointNames;

        /// \brief Calculated speed of rear left wheel joint(s)
        double rearLeftJointSpeed{0};

        /// \brief Calculated speed of rear right wheel joint(s)
        double rearRightJointSpeed{0};

        /// \brief Front steering wheels
        std::array<std::unique_ptr<FrontWheel>, 4> frontSteeringWheels;

        /// \brief Left wheel rotation joints
        std::vector<Entity> rearLeftJoints;

        /// \brief Right wheel rotation joints
        std::vector<Entity> rearRightJoints;

        /// \brief A mutex to protect the target velocity command.
        std::mutex mutex;

        /// \brief Last target velocity requested.
        msgs::Twist targetVel;

        /// \brief Previous control command. 
        Commands last0Cmd; //v(k-1), w(k-1)

        /// \brief Previous control command to last0Cmd.
        Commands last1Cmd; //v(k-2), w(k-2)

        /// \brief Derived geometry parameters.
        DerivedGeometry der_geom;

        /// \brief Odometry X value
        double odomX{0.0};

        /// \brief Odometry Y value
        double odomY{0.0};

        /// \brief Odometry yaw value
        double odomYaw{0.0};

        /// \brief Odometry old left value
        double odomOldLeft{0.0};

        /// \brief Odometry old right value
        double odomOldRight{0.0};

        /// \brief Odometry last time value
        std::chrono::steady_clock::duration lastOdomTime{0};

        /// \brief Update period calculated from <odom__publish_frequency>.
        std::chrono::steady_clock::duration odomPubPeriod{0};

        /// \brief Last sim time odom was published.
        std::chrono::steady_clock::duration lastOdomPubTime{0};

        /// \brief frame_id from sdf.
        string sdfFrameId;

        /// \brief child_frame_id from sdf.
        string sdfChildFrameId;

        /// \brief Truck steering odometry message publisher.
        transport::Node::Publisher odomPub;

        /// \brief Truck tf message publisher.
        transport::Node::Publisher tfPub;

        /// \brief Callback for velocity subscription
        /// \param[in] _msg Velocity message
        void OnCmdVel(const msgs::Twist &msg);

        /// \brief Update the linear and angular velocities.
        /// \param[in] _info System update information.
        /// \param[in] _ecm The EntityComponentManager of the given simulation
        /// instance.
        void UpdateVelocity(const UpdateInfo &info, const EntityComponentManager &ecm);

        void UpdateOdometry(const UpdateInfo &info, const EntityComponentManager &ecm);

        bool missingFrontWheelJoints() const;
        bool missingFrontWheelJointNames() const;

};

bool TruckPluginPrivate::missingFrontWheelJoints() const
{
    for (const auto& wheel : this->frontSteeringWheels)
        if (wheel->missingSteerJoint() || (this->drivenWheels == 8 && wheel->missingDriveJoint()))
            return true;

    return false;
}


bool TruckPluginPrivate::missingFrontWheelJointNames() const
{
    for (const auto& wheel : this->frontSteeringWheels)
        if (wheel->steerJointName.empty() || (this->drivenWheels == 8 && wheel->driveJointName.empty()))
            return true;

    return false;
}


//////////////////////////////////////////////////
void TruckPluginPrivate::OnCmdVel(const msgs::Twist &msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetVel = msg;
}



//////////////////////////////////////////////////
void TruckPluginPrivate::UpdateVelocity(const UpdateInfo &info, const EntityComponentManager &ecm)
{
    GZ_PROFILE("TruckPlugin::UpdateVelocity");

    double linVel, angVel;
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        linVel = this->targetVel.linear().x();
        angVel = this->targetVel.angular().z();
    }

    //last0 and last1 are necessary to limit acceleration and jerk
    this->limiterLin->Limit(linVel, this->last0Cmd.lin, this->last1Cmd.lin, info.dt);
    this->limiterAng->Limit(angVel, this->last0Cmd.ang, this->last1Cmd.ang, info.dt);

    // Update history of commands.
    this->last1Cmd = last0Cmd;
    this->last0Cmd.lin = linVel;
    this->last0Cmd.ang = angVel;

    double turningRadius;
    double minimumTurningRadius = this->der_geom.Ld1 / tan(this->steeringLimit);

    if (fabs(angVel) < 1e-3)
        turningRadius = 1000000000.0;

    else
    {
        turningRadius = linVel / angVel;

        if (fabs(linVel) > 0.0)
        {
            if(fabs(turningRadius) < minimumTurningRadius)
                turningRadius = minimumTurningRadius * (turningRadius >= 0.0 ? 1.0 : -1.0);
        }

        else
            turningRadius = angVel/fabs(angVel) * minimumTurningRadius;
    }

    double steerAngle[4];
    steerAngle[FL1] = atan(this->der_geom.Ld1 / (turningRadius - this->frontWheelSeparation / 2.0));
    steerAngle[FR1] = atan(this->der_geom.Ld1 / (turningRadius + this->frontWheelSeparation / 2.0));
    steerAngle[FL2] = atan(this->der_geom.Ld2 / (turningRadius - this->frontWheelSeparation / 2.0));
    steerAngle[FR2] = atan(this->der_geom.Ld2 / (turningRadius + this->frontWheelSeparation / 2.0));

    double leftDist = 1.0 - this->rearWheelSeparation / (2.0 * turningRadius);
    double rightDist = 1.0 + this->rearWheelSeparation / (2.0 * turningRadius);
    this->rearLeftJointSpeed = (linVel / this->wheelRadius) * sqrt(pow(leftDist, 2) + pow(this->der_geom.dc/turningRadius, 2));
    this->rearRightJointSpeed = (linVel / this->wheelRadius) * sqrt(pow(rightDist, 2) + pow(this->der_geom.dc/turningRadius, 2));

    if (this->drivenWheels == 8)
    {
        double frontLeftDist = 1.0 - this->frontWheelSeparation / (2.0 * turningRadius);
        double frontRightDist = 1.0 + this->frontWheelSeparation / (2.0 * turningRadius);

        this->frontSteeringWheels[FL1]->driveSpeed = (linVel / this->wheelRadius) * sqrt(pow(frontLeftDist, 2) + pow(this->der_geom.Ld1/turningRadius, 2));
        this->frontSteeringWheels[FR1]->driveSpeed = (linVel / this->wheelRadius) * sqrt(pow(frontRightDist, 2) + pow(this->der_geom.Ld1/turningRadius, 2));
        this->frontSteeringWheels[FL2]->driveSpeed = (linVel / this->wheelRadius) * sqrt(pow(frontLeftDist, 2) + pow(this->der_geom.Ld2/turningRadius, 2));
        this->frontSteeringWheels[FR2]->driveSpeed = (linVel / this->wheelRadius) * sqrt(pow(frontRightDist, 2) + pow(this->der_geom.Ld2/turningRadius, 2));
    }
    

    std::vector<const components::JointPosition*> steeringPositions(4);
    for (int i = 0; i < 4; ++i)
        steeringPositions[i] = ecm.Component<components::JointPosition>(this->frontSteeringWheels[i]->steerJoint);


    for (int i = 0; i < 4; ++i)
        if(!steeringPositions[i] || steeringPositions[i]->Data().empty())
            return;


    double delta[4];
    for (int i = 0; i < 4; ++i)
        delta[i] = steerAngle[i] - steeringPositions[i]->Data()[0];

    // Simple proportional control with settable gain.
    // Adding programmable PID values might be a future feature.
    for (int i = 0; i < 4; ++i)
        this->frontSteeringWheels[i]->steerSpeed = this->gainPAng * delta[i];
}



//////////////////////////////////////////////////
void TruckPluginPrivate::UpdateOdometry(const UpdateInfo & info, const EntityComponentManager & ecm)
{
    GZ_PROFILE("TruckPlugin::UpdateOdometry");

    // \TODO(anyone) Implement odometry calculation and publishing

    if (this->rearLeftJointNames.empty() || this->rearRightJointNames.empty() || this->missingFrontWheelJointNames())
    {
        gzerr << "No valid joints specified for TruckPlugin. Failed to update." << std::endl;
        return;
    }

    // Get the first joint positions for the left and right side.

    auto collectPositions = [&] (const std::vector<Entity>& joints, std::vector<const components::JointPosition*>& positions) -> bool
    {
        positions.clear();
        positions.reserve(joints.size());

        for (const auto& joint: joints)
        {
            auto pos = ecm.Component<components::JointPosition>(joint);
            if (!pos || pos->Data().empty())
                return false;

            positions.push_back(pos);
        }
        return true;
    }; 

    auto equivalentSteeringAngle = [] (double deltaLeft, double deltaRight)
    {
        const double tanL = tan(deltaLeft);
        const double tanR = tan(deltaRight);
        const double denom = tanL + tanR;

        if (abs(denom) < 1e-9)
            return 0.0;

        const double tanCenter = 2.0 * tanL * tanR / denom;
        return atan(tanCenter);
    };


    std::vector<Entity> frontSteerJoints, frontDriveJoints;
    std::vector<const components::JointPosition*> leftPos, rightPos, frontSteerPos, frontDrivePos;

    frontSteerJoints.reserve(4);
    if(this->drivenWheels == 8)
        frontDriveJoints.reserve(4);

    for (const auto &wheel : this->frontSteeringWheels)
    {
        frontSteerJoints.push_back(wheel->steerJoint);

        if (this->drivenWheels == 8)
            frontDriveJoints.push_back(wheel->driveJoint);
    }

    // Abort if the joints were not found or just created.
    if (!collectPositions(this->rearLeftJoints, leftPos) || !collectPositions(this->rearRightJoints, rightPos) ||
        !collectPositions(frontSteerJoints, frontSteerPos) || (this->drivenWheels == 8 && !collectPositions(frontDriveJoints, frontDrivePos)))
        return;

    double phi1 = equivalentSteeringAngle(frontSteerPos[FL1]->Data()[0], frontSteerPos[FR1]->Data()[0]);
    double phi2 = equivalentSteeringAngle(frontSteerPos[FL2]->Data()[0], frontSteerPos[FR2]->Data()[0]);

    double radius1 = this->der_geom.Ld1 / tan(phi1);
    double radius2 = this->der_geom.Ld2 / tan(phi2);
    double radius = 0.5 * (radius1 + radius2);

    double leftMean = 0.5 * (leftPos[0]->Data()[0] + leftPos[1]->Data()[0]);
    double rightMean = 0.5 * (rightPos[0]->Data()[0] + rightPos[1]->Data()[0]);

    double dist = 0.5 * this->wheelRadius * ((leftMean - this->odomOldLeft) + (rightMean - this->odomOldRight));
    double deltaAngle = dist / radius;

    double yawMid = this->odomYaw + deltaAngle / 2.0;
    this->odomX += dist * cos(yawMid);
    this->odomY += dist * sin(yawMid);
    this->odomYaw += deltaAngle;
    this->odomYaw = math::Angle(this->odomYaw).Normalized().Radian();

    auto odomTimeDiff = info.simTime - this->lastOdomTime;
    double tdiff = std::chrono::duration<double>(odomTimeDiff).count();
    double odomLinearVelocity = dist / tdiff;
    double odomAngularVelocity = deltaAngle / tdiff;
    this->lastOdomTime = info.simTime;
    this->odomOldLeft = leftMean;
    this->odomOldRight = rightMean;

    // Throttle odometry publishing
    auto diff = info.simTime - this->lastOdomPubTime;
    if (diff > std::chrono::steady_clock::duration::zero() && diff < this->odomPubPeriod)
        return;

    this->lastOdomPubTime = info.simTime;

    // Construct the odometry message and publish it.
    msgs::Odometry msg;
    msg.mutable_pose()->mutable_position()->set_x(this->odomX);
    msg.mutable_pose()->mutable_position()->set_y(this->odomY);

    math::Quaterniond orientation(0, 0, this->odomYaw);
    msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

    msg.mutable_twist()->mutable_linear()->set_x(odomLinearVelocity);
    msg.mutable_twist()->mutable_angular()->set_z(odomAngularVelocity);

    // Set the frame id.
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");

    if (this->sdfFrameId.empty())
        frame->add_value(this->model.Name(ecm) + "/odom");

    else
        frame->add_value(this->sdfFrameId);

    std::optional<std::string> linkName = this->canonicalLink.Name(ecm);
    if (this->sdfChildFrameId.empty())
    {
        if (linkName)
        {
            auto childFrame = msg.mutable_header()->add_data();
            childFrame->set_key("child_frame_id");
            childFrame->add_value(this->model.Name(ecm) + "/" + *linkName);
        }
    }

    else
    {
        auto childFrame = msg.mutable_header()->add_data();
        childFrame->set_key("child_frame_id");
        childFrame->add_value(this->sdfChildFrameId);
    }

    // Construct the Pose_V/tf message and publish it.
    msgs::Pose_V tfMsg;
    msgs::Pose *tfMsgPose = tfMsg.add_pose();
    tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
    tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
    tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

    // Publish the message
    this->odomPub.Publish(msg);
    this->tfPub.Publish(tfMsg);
}



//////////////////////////////////////////////////
TruckPlugin::TruckPlugin(): dataPtr(std::make_unique<TruckPluginPrivate>())
{
}



//////////////////////////////////////////////////
void TruckPlugin::Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf, EntityComponentManager &ecm, EventManager &/**/)
{
    //lambda function
    auto getSdfParam = [&] (const string &name, double defaultValue) -> double
    {
        auto [value, found] = sdf->Get<double>(name, defaultValue);
        if (!found)
            gzerr << name << " not specified in SDF, using default value: " << defaultValue << std::endl;
        
        return value;
    };

    auto getRequiredString = [&] (const string &tag) -> string
    {
        if (!sdf->HasElement(tag))
        {
            gzerr << "Missing required SDF tag <" << tag << ">." << std::endl;
            return "";
        }

        auto value = sdf->Get<std::string>(tag);
        if (value.empty())
            gzerr << "Empty value for required SDF tag <" << tag << ">." << std::endl;

        return value;
    };
    
    
    this->dataPtr->model = Model(entity); //crea un oggetto Model a partire dall'entity (robot_description) a cui è attaccato il plugin

    if (!this->dataPtr->model.Valid(ecm)) //qui controlla se l'entity è effettivamente un modello
    {
        gzerr << "TruckPlugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }

    // creaimo un vettore di entità che contengono i nostri link, inserendo l'entità che stiamo trattando e i link di tale entità
    // e poi prendiamo il primo che troviamo con il componente CanonicalLink, che è quello che ci serve per calcolare l'odometria
    std::vector<Entity> links = ecm.ChildrenByComponents( this->dataPtr->model.Entity(), components::CanonicalLink());
    gzmsg << "Found [" << links.size() << "] canonical links for model [" << this->dataPtr->model.Name(ecm) << "]." << std::endl;
    if (!links.empty())
        this->dataPtr->canonicalLink = Link(links[0]);

    this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>(); //limiti di velocità lineare
    this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>(); //limiti di velocità angolare

    //con HasElement controlliamo se esiste il parametro, se non esiste ignoriamo completamente l'inizializzazione
    //altrimenti lo inizializziamo con il valore specificato. Diverso da Get, perché se non è specificato lo
    // inizializziamo comunque col valore di default, ma con un messaggio di warning. 
    if(sdf->HasElement("max_velocity"))
    {
        const double maxVel = sdf->Get<double>("max_velocity");
        this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
        this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
    }

    else
        gzwarn << "Maximum velocity not specified in SDF" << std::endl;

    if(sdf->HasElement("min_velocity"))
    {
        const double minVel = sdf->Get<double>("min_velocity");
        this->dataPtr->limiterLin->SetMinVelocity(minVel);
        this->dataPtr->limiterAng->SetMinVelocity(minVel);
    }

    else
        gzwarn << "Minimum velocity not specified in SDF" << std::endl;

    if(sdf->HasElement("max_acceleration"))
    {
        const double maxAcc = sdf->Get<double>("max_acceleration");
        this->dataPtr->limiterLin->SetMaxAcceleration(maxAcc);
        this->dataPtr->limiterAng->SetMaxAcceleration(maxAcc);
    }

    else
        gzwarn << "Maximum acceleration not specified in SDF" << std::endl;

    if(sdf->HasElement("min_acceleration"))
    {
        const double minAcc = sdf->Get<double>("min_acceleration");
        this->dataPtr->limiterLin->SetMinAcceleration(minAcc);
        this->dataPtr->limiterAng->SetMinAcceleration(minAcc);
    }

    else
        gzwarn << "Minimum acceleration not specified in SDF" << std::endl;

    if(sdf->HasElement("steer_p_gain"))
        this->dataPtr->gainPAng = sdf->Get<double>("steer_p_gain");

    else
        gzwarn << "Steering proportional gain not specified in SDF" << std::endl;

    this->dataPtr->wheelRadius = getSdfParam("wheel_radius", this->dataPtr->wheelRadius);

    this->dataPtr->rearWheelBase = getSdfParam("rear_wheel_base", this->dataPtr->rearWheelBase);

    this->dataPtr->frontWheelBase = getSdfParam("front_wheel_base", this->dataPtr->frontWheelBase);

    this->dataPtr->centralWheelBase = getSdfParam("central_wheel_base", this->dataPtr->centralWheelBase);

    this->dataPtr->steeringLimit = getSdfParam("steering_limit", this->dataPtr->steeringLimit);

    this->dataPtr->rearWheelSeparation = getSdfParam("rear_wheel_separation", this->dataPtr->rearWheelSeparation);

    this->dataPtr->frontWheelSeparation = getSdfParam("front_wheel_separation", this->dataPtr->frontWheelSeparation);

    this->dataPtr->drivenWheels = getSdfParam("driven_wheels", this->dataPtr->drivenWheels);
    if (this->dataPtr->drivenWheels != 4 && this->dataPtr->drivenWheels != 8)
    {
        gzerr << "Invalid number of driven wheels specified: " << this->dataPtr->drivenWheels
              << ". Supported values are 4 and 8. Defaulting to 4." << std::endl;
        this->dataPtr->drivenWheels = 4;
    }

    this->dataPtr->der_geom.dc = this->dataPtr->rearWheelBase / 2.0;
    this->dataPtr->der_geom.Ld2 = this->dataPtr->centralWheelBase + this->dataPtr->der_geom.dc;
    this->dataPtr->der_geom.Ld1 = this->dataPtr->frontWheelBase + this->dataPtr->der_geom.Ld2;
    
    //prendiamo i nomi dei giunti dall'SDF e poi le entità corrispondenti
    auto sdfLeftElem = sdf->FindElement("rear_left_joint");
    while (sdfLeftElem)
    {
        auto name = sdfLeftElem->Get<string>();
        //gzmsg << "Loaded left joint name from SDF: " << name << std::endl;
        this->dataPtr->rearLeftJointNames.push_back(name);
        sdfLeftElem = sdfLeftElem->GetNextElement("rear_left_joint");
    }

    if (this->dataPtr->rearLeftJointNames.size() != 2)
    {
        gzerr << "TruckPlugin: expected exactly 2 <rear_left_joint> entries, found "
              << this->dataPtr->rearLeftJointNames.size() << "." << std::endl;
        return;
    }

    auto sdfRightElem = sdf->FindElement("rear_right_joint");
    while (sdfRightElem)
    {
        auto name = sdfRightElem->Get<string>();
        //gzmsg << "Loaded right joint name from SDF: " << name << std::endl;
        this->dataPtr->rearRightJointNames.push_back(name);
        sdfRightElem = sdfRightElem->GetNextElement("rear_right_joint");
    }

    if (this->dataPtr->rearRightJointNames.size() != 2)
    {
        gzerr << "TruckPlugin: expected exactly 2 <rear_right_joint> entries, found "
              << this->dataPtr->rearRightJointNames.size() << "." << std::endl;
        return;
    }


    for (auto &wheel : this->dataPtr->frontSteeringWheels)
        wheel = std::make_unique<FrontWheel>();

    this->dataPtr->frontSteeringWheels[FL1]->steerJointName = getRequiredString("front_left_1_steering_joint");
    this->dataPtr->frontSteeringWheels[FR1]->steerJointName = getRequiredString("front_right_1_steering_joint");
    this->dataPtr->frontSteeringWheels[FL2]->steerJointName = getRequiredString("front_left_2_steering_joint");
    this->dataPtr->frontSteeringWheels[FR2]->steerJointName = getRequiredString("front_right_2_steering_joint");

    if (this->dataPtr->drivenWheels == 8)
    {
        this->dataPtr->frontSteeringWheels[FL1]->driveJointName = getRequiredString("front_left_1_drive_joint");
        this->dataPtr->frontSteeringWheels[FR1]->driveJointName = getRequiredString("front_right_1_drive_joint");
        this->dataPtr->frontSteeringWheels[FL2]->driveJointName = getRequiredString("front_left_2_drive_joint");
        this->dataPtr->frontSteeringWheels[FR2]->driveJointName = getRequiredString("front_right_2_drive_joint");
    }


    double odomFreq = getSdfParam("odom_publish_frequency", 50.0);
    if (odomFreq > 0)
    {
        std::chrono::duration<double> odomPer{1 / odomFreq};
        this->dataPtr->odomPubPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
    }

    std::vector<string> topics;

    if(sdf->HasElement("topic"))
        topics.push_back(sdf->Get<string>("topic"));

    else if(sdf->HasElement("sub_topic"))
        topics.push_back("/model/" + this->dataPtr->model.Name(ecm) + "/" + sdf->Get<string>("sub_topic"));

    else
        topics.push_back("/model/" + this->dataPtr->model.Name(ecm) + "/cmd_vel");


    auto topic = validTopic(topics);
    if (topic.empty())
    {
        gzerr << "No valid topic specified for TruckPlugin. Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->node.Subscribe(topic, &TruckPluginPrivate::OnCmdVel, this->dataPtr.get());
    gzmsg << "TruckPlugin subscribing to twist messages on [" << topic << "]" << std::endl;

    std::vector<string> odomTopics;
    if (sdf->HasElement("odom_topic"))
        odomTopics.push_back(sdf->Get<string>("odom_topic"));


    odomTopics.push_back("/model/" + this->dataPtr->model.Name(ecm) + "/odometry");
    auto odomTopic = validTopic(odomTopics);
    if (topic.empty())
    {
        gzerr << "AckermannSteering plugin received invalid model name " << "Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(odomTopic);

    std::vector<string> tfTopics;
    if (sdf->HasElement("tf_topic"))
        tfTopics.push_back(sdf->Get<string>("tf_topic"));

    tfTopics.push_back("/model/" + this->dataPtr->model.Name(ecm) + "/tf");
    auto tfTopic = validTopic(tfTopics);
    if (tfTopic.empty())
    {
        gzerr << "AckermannSteering plugin invalid tf topic name " << "Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(tfTopic);


    if (sdf->HasElement("frame_id"))
        this->dataPtr->sdfFrameId = sdf->Get<string>("frame_id");

    if (sdf->HasElement("child_frame_id"))
        this->dataPtr->sdfChildFrameId = sdf->Get<string>("child_frame_id");
    
}


//////////////////////////////////////////////////
void TruckPlugin::PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
    GZ_PROFILE("TruckPlugin::PreUpdate");

    // \TODO(anyone) Support rewind
    if (info.dt < std::chrono::steady_clock::duration::zero())
        gzwarn << "Detected jump back in time ["
               << std::chrono::duration<double>(info.dt).count() 
               << "s]. System may not work properly." << std::endl;

    
    static std::set<string> warnedModels;
    auto modelName = this->dataPtr->model.Name(ecm);

    if (this->dataPtr->rearLeftJoints.empty() || this->dataPtr->rearRightJoints.empty())
    {
        bool warned = false;
        for(const string& name : this->dataPtr->rearLeftJointNames)
        {
            Entity joint = this->dataPtr->model.JointByName(ecm, name);
            if (joint != kNullEntity)
                this->dataPtr->rearLeftJoints.push_back(joint);

            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                gzwarn << "Failed to find left joint [" << name << "] for model [" << modelName << "]" << std::endl;
                warned = true;
            }
        }
        
        
        for(const string& name : this->dataPtr->rearRightJointNames)
        {
            Entity joint = this->dataPtr->model.JointByName(ecm, name);
            if (joint != kNullEntity)
                this->dataPtr->rearRightJoints.push_back(joint);

            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                gzwarn << "Failed to find right joint [" << name << "] for model [" << modelName << "]" << std::endl;
                warned = true;
            }
        }

        if(warned)
            warnedModels.insert(modelName);
    }


///////////////////////////
    if (this->dataPtr->missingFrontWheelJoints())
    {
        bool warned = false;

        for(int i=0; i < 4; ++i)
        {
            Entity joint = this->dataPtr->model.JointByName(ecm, this->dataPtr->frontSteeringWheels[i]->steerJointName);

            if (joint != kNullEntity)
            {
                this->dataPtr->frontSteeringWheels[i]->steerJoint = joint;
                gzmsg << "Found first steering joint [" << this->dataPtr->frontSteeringWheels[i]->steerJointName << "] for model [" << modelName << "]." << std::endl;
            }

            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                gzwarn << "Failed to find first steering joint [" << this->dataPtr->frontSteeringWheels[i]->steerJointName << "] for model [" << modelName << "]" << std::endl;
                warned = true;
            }

            if (this->dataPtr->drivenWheels == 8)
            {
                Entity joint = this->dataPtr->model.JointByName(ecm, this->dataPtr->frontSteeringWheels[i]->driveJointName);

                if (joint != kNullEntity)
                {
                    this->dataPtr->frontSteeringWheels[i]->driveJoint = joint;
                    gzmsg << "Found drive joint [" << this->dataPtr->frontSteeringWheels[i]->driveJointName << "] for model [" << modelName << "]." << std::endl;
                }

                else if (warnedModels.find(modelName) == warnedModels.end())
                {
                    gzwarn << "Failed to find drive joint [" << this->dataPtr->frontSteeringWheels[i]->driveJointName << "] for model [" << modelName << "]" << std::endl;
                    warned = true;
                }
            }
        }

        if(warned)
            warnedModels.insert(modelName);
    }


    if (this->dataPtr->rearLeftJoints.empty() || this->dataPtr->rearRightJoints.empty() || this->dataPtr->missingFrontWheelJoints())
    {
        gzerr << "No valid joints specified for TruckPlugin. Failed to update." << std::endl;
        return;
    }


    if (warnedModels.find(modelName) != warnedModels.end())
    {
        gzmsg << "Found joints for model [" << modelName << "], plugin will start working." << std::endl;
        warnedModels.erase(modelName);
    }


    // Nothing left to do if paused.
    if (info.paused)
        return;

    // Update wheel velocity
    for (Entity joint : this->dataPtr->rearLeftJoints)
    {
        //gzmsg << "Set left joint  velocity to " << this->dataPtr->rearLeftJointSpeed << std::endl;
        ecm.SetComponentData<components::JointVelocityCmd>(joint, {this->dataPtr->rearLeftJointSpeed});
    }

    for (Entity joint : this->dataPtr->rearRightJoints)
    {
        //gzmsg << "Set right joint  velocity to " << this->dataPtr->rearRightJointSpeed << std::endl;
        ecm.SetComponentData<components::JointVelocityCmd>(joint, {this->dataPtr->rearRightJointSpeed});
    }

    if(this->dataPtr->drivenWheels == 8)
    {
        for (const auto& wheel : this->dataPtr->frontSteeringWheels)
            if (wheel->driveJoint != kNullEntity)
                ecm.SetComponentData<components::JointVelocityCmd>(wheel->driveJoint, {wheel->driveSpeed});
    }


    for (const auto& wheel : this->dataPtr->frontSteeringWheels)
        if (wheel->steerJoint != kNullEntity)
            ecm.SetComponentData<components::JointVelocityCmd>(wheel->steerJoint, {wheel->steerSpeed});
    
    // Create the left and right side joint position components if they
    // don't exist.
    for(long unsigned int i=0; i < this->dataPtr->rearLeftJoints.size(); ++i)
    {
        auto leftPos = ecm.Component<components::JointPosition>(this->dataPtr->rearLeftJoints[i]);
        if (!leftPos)
            ecm.CreateComponent(this->dataPtr->rearLeftJoints[i], components::JointPosition());
    }
    
    for(long unsigned int i=0; i < this->dataPtr->rearRightJoints.size(); ++i)
    {
        auto rightPos = ecm.Component<components::JointPosition>(this->dataPtr->rearRightJoints[i]);
        if (!rightPos)
            ecm.CreateComponent(this->dataPtr->rearRightJoints[i], components::JointPosition());
    }


    for(const auto& wheel: this->dataPtr->frontSteeringWheels)
    {
        auto wheelSteerPos = ecm.Component<components::JointPosition>(wheel->steerJoint);
        if (!wheelSteerPos)
            ecm.CreateComponent(wheel->steerJoint, components::JointPosition());

        if (this->dataPtr->drivenWheels == 8)
        {
            auto wheelDrivePos = ecm.Component<components::JointPosition>(wheel->driveJoint);
            if (!wheelDrivePos)
                ecm.CreateComponent(wheel->driveJoint, components::JointPosition());
        }
    }
}



void TruckPlugin::PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm)
{
    GZ_PROFILE("TruckPlugin::PostUpdate");

    // \TODO(anyone) Support rewind
    if (info.paused)
        return;


    if (this->dataPtr->missingFrontWheelJoints())
        return;

    this->dataPtr->UpdateVelocity(info, ecm);
    this->dataPtr->UpdateOdometry(info, ecm);
}



GZ_ADD_PLUGIN(
    TruckPlugin,
    System,
    TruckPlugin::ISystemConfigure,
    TruckPlugin::ISystemPreUpdate,
    TruckPlugin::ISystemPostUpdate)
    