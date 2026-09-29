#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <cmath>
#include <random>
#include <memory>
#include <mutex>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

namespace imu_pub
{
  class ImuPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &) override
    {
      gz::sim::Model model(_entity);
      std::string linkName = "base_link";
      if (_sdf->HasElement("link_name"))
        linkName = _sdf->Get<std::string>("link_name");
      this->link = gz::sim::Link(model.LinkByName(_ecm, linkName));

      std::string prefix = "/" + model.Name(_ecm);
      this->pub = this->node.Advertise<gz::msgs::IMU>(prefix + "/imu");
      this->magPub = this->node.Advertise<gz::msgs::Magnetometer>(prefix + "/mag");
      this->magBias.Set(0.0, 0.0, 0.0);
      if (_sdf->HasElement("mag_north_uT")) {
        double north = _sdf->Get<double>("mag_north_uT");
        this->magBias.Set(north, 0.0, 0.0);
      }
      if (_sdf->HasElement("mag_down_uT")) {
        double down = _sdf->Get<double>("mag_down_uT");
        magBias.Z() = down;
      }
      if (_sdf->HasElement("mag_noise_stddev")) {
        this->magNoiseSigma = _sdf->Get<double>("mag_noise_stddev");
      }
      if (_sdf->HasElement("accel_noise_stddev")) {
        this->accelNoiseSigma = _sdf->Get<double>("accel_noise_stddev");
      }
      gzdbg << "[ImuPlugin] configured, link=" << linkName
            << " topic=" << prefix << "/imu"
            << " mag=" << prefix << "/mag"
            << " mag_bias_uT=(" << magBias.X() << "," << magBias.Y() << "," << magBias.Z() << ")"
            << std::endl;
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override
    {
      auto pose = this->link.WorldPose(_ecm);
      if (!pose) {
        if (++this->skip < 4)
          gzdbg << "[ImuPlugin] no world pose for link" << std::endl;
        return;
      }
      auto angVel = this->link.WorldAngularVelocity(_ecm);
      auto linVel = this->link.WorldLinearVelocity(_ecm);

      double t = std::chrono::duration<double>(_info.simTime).count();
      gz::math::Vector3d av, lv;
      if (angVel && linVel) {
        av = *angVel;
        lv = *linVel;
      } else if (this->prevT >= 0 && t > this->prevT) {
        double dt = t - this->prevT;
        lv = (pose->Pos() - this->prevPos) / dt;
        gz::math::Quaterniond q = pose->Rot();
        gz::math::Quaterniond dq = q * this->prevQ.Inverse();
        double ang = 2.0 * std::acos(std::max(-1.0, std::min(1.0, dq.W())));
        if (dq.W() < 0.0) ang = -ang;
        gz::math::Vector3d axis(0, 0, 1);
        if (dq.X() != 0 || dq.Y() != 0 || dq.Z() != 0)
          axis = gz::math::Vector3d(dq.X(), dq.Y(), dq.Z()).Normalized();
        av = axis * (ang / dt);
      } else {
        av.Set(0, 0, 0);
        lv.Set(0, 0, 0);
      }
      this->prevPos = pose->Pos();
      this->prevQ = pose->Rot();
      this->prevT = t;

      gz::math::Quaterniond qWorld = pose->Rot();

      // WorldAngularVelocity is expressed in the world frame; the estimator and
      // attitude loop expect body-frame gyro: w_body = R^T * w_world.
      gz::math::Vector3d avBody = qWorld.RotateVectorReverse(av);
      // The Gazebo body frame is x-forward / y-left / z-up (SDF default). The
      // controller follows the PX4 NED/FRD convention (x fwd, y right, z down),
      // so every body-frame signal is rotated 180 deg about x: flip Y and Z.
      gz::math::Vector3d avFrd(avBody.X(), -avBody.Y(), -avBody.Z());

      // True body-frame specific force: a_body = R^T * (dv/dt_W - g_W), g_W = (0,0,-9.81).
      gz::math::Vector3d aWorld;
      if (this->prevLvValid && t > this->prevLvT) {
        double dt = t - this->prevLvT;
        aWorld = (lv - this->prevLv) / dt;
      } else {
        // No velocity history yet -> assume leveled stationary (noise floor).
        aWorld.Set(0, 0, 0);
      }
      this->prevLv = lv;
      this->prevLvT = t;
      this->prevLvValid = true;

      // In ENU the gravity vector is (0,0,-9.81); specific force = a - g = (0,0,+9.81) when resting.
      gz::math::Vector3d gWorld(0.0, 0.0, -9.81);
      gz::math::Vector3d specificWorld = aWorld - gWorld;
      gz::math::Vector3d specificBody = qWorld.RotateVectorReverse(specificWorld);
      gz::math::Vector3d specificFrd(specificBody.X(), -specificBody.Y(), -specificBody.Z());

      // Magnetic field: fixed world-frame field (north +X, down -Z, microtesla),
      // rotated into the body frame, plus gaussian noise.
      gz::math::Vector3d magWorld(this->magBias.X(), this->magBias.Y(), this->magBias.Z());
      gz::math::Vector3d magBody = qWorld.RotateVectorReverse(magWorld);
      gz::math::Vector3d magFrd(magBody.X(), -magBody.Y(), -magBody.Z());
      magFrd.X() += this->noise() * this->magNoiseSigma;
      magFrd.Y() += this->noise() * this->magNoiseSigma;
      magFrd.Z() += this->noise() * this->magNoiseSigma;

      gz::msgs::IMU msg;
      msg.mutable_header()->mutable_stamp()->set_sec(
          std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
      msg.mutable_header()->mutable_stamp()->set_nsec(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              _info.simTime % std::chrono::seconds(1))
              .count());
      msg.set_entity_name(this->link.Name(_ecm).value_or(""));

      auto *q = msg.mutable_orientation();
      q->set_x(pose->Rot().X());
      q->set_y(pose->Rot().Y());
      q->set_z(pose->Rot().Z());
      q->set_w(pose->Rot().W());
      msg.mutable_angular_velocity()->set_x(avFrd.X());
      msg.mutable_angular_velocity()->set_y(avFrd.Y());
      msg.mutable_angular_velocity()->set_z(avFrd.Z());
      msg.mutable_linear_acceleration()->set_x(specificFrd.X() + this->noise() * this->accelNoiseSigma);
      msg.mutable_linear_acceleration()->set_y(specificFrd.Y() + this->noise() * this->accelNoiseSigma);
      msg.mutable_linear_acceleration()->set_z(specificFrd.Z() + this->noise() * this->accelNoiseSigma);

      gz::msgs::Magnetometer mag;
      *mag.mutable_header() = msg.header();
      mag.mutable_field_tesla()->set_x(magFrd.X() * 1.0e-6);
      mag.mutable_field_tesla()->set_y(magFrd.Y() * 1.0e-6);
      mag.mutable_field_tesla()->set_z(magFrd.Z() * 1.0e-6);

      this->pub.Publish(msg);
      this->magPub.Publish(mag);
    }

    private: double noise()
    {
      std::normal_distribution<double> dist(0.0, 1.0);
      return dist(this->rng);
    }

    private: gz::sim::Link link;
    private: unsigned int skip = 0;
    private: double prevT = -1.0;
    private: gz::math::Vector3d prevPos;
    private: gz::math::Quaterniond prevQ;
    private: bool prevLvValid = false;
    private: double prevLvT = -1.0;
    private: gz::math::Vector3d prevLv;
    private: gz::math::Vector3d magBias;
    private: double magNoiseSigma = 0.1;   // uT per axis
    private: double accelNoiseSigma = 0.05; // m/s^2 per axis
    private: std::mt19937 rng{42};
    private: gz::transport::Node node;
    private: gz::transport::Node::Publisher pub;
    private: gz::transport::Node::Publisher magPub;
  };
}

GZ_ADD_PLUGIN(imu_pub::ImuPlugin, gz::sim::System,
              imu_pub::ImuPlugin::ISystemConfigure,
              imu_pub::ImuPlugin::ISystemPreUpdate)