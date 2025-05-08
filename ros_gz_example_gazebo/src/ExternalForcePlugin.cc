#include "ros_gz_example_gazebo/ExternalForcePlugin.hh"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void systems::ExternalForcePlugin::Configure(const Entity &_entity,
                                              const std::shared_ptr<const sdf::Element> &_sdf,
                                              EntityComponentManager &_ecm,
                                              EventManager &_eventMgr)
{
  this->modelEntity = _entity;

  // SDF에서 Joint 이름 가져오기
  auto jointName = _sdf->Get<std::string>("joint_name", "").first;
  if (jointName.empty())
  {
    gzerr << "Joint name is required in SDF <joint_name> tag.\n";
    return;
  }

  // Joint 엔티티 찾기
  this->jointEntity = _ecm.EntityByComponents(components::Joint(jointName));
  if (!this->jointEntity)
  {
    gzerr << "Joint [" << jointName << "] not found.\n";
    return;
  }

  // SDF에서 Topic 이름 가져오기
  auto topicName = _sdf->Get<std::string>("topic", "/external_force").first;

  // Gazebo 메시지 게시자 설정
  this->forcePub = this->node.Advertise<msgs::Wrench>(topicName);

  gzdbg << "External Force Plugin configured for joint [" << jointName
        << "] with topic [" << topicName << "].\n";
}

//////////////////////////////////////////////////
void systems::ExternalForcePlugin::PreUpdate(const UpdateInfo &_info,
                                             EntityComponentManager &_ecm)
{
  // 엔티티의 Pose 가져오기
  auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
  if (!poseComp)
  {
    gzerr << "Pose component not found.\n";
    return;
  }

  auto pose = poseComp->Data(); // Pose3d 객체
  gzdbg << "Entity Pose: [" << pose.Pos().X() << ", "
        << pose.Pos().Y() << ", " << pose.Pos().Z() << "]\n";

  // 측정된 힘 데이터 예제
  math::Vector3d measuredForce{10.0, 5.0, 15.0}; // 가상의 데이터

  // 힘 변화 계산 및 Baseline 업데이트
  double deltaForce = (measuredForce - this->baselineForce).Length();
  if (deltaForce < this->forceThreshold)
  {
    this->baselineForce = measuredForce;
  }
  else
  {
    math::Vector3d externalForce = measuredForce - this->baselineForce;

    // 외력 데이터 메시지 생성
    msgs::Wrench wrenchMsg;
    wrenchMsg.mutable_force()->set_x(externalForce.X());
    wrenchMsg.mutable_force()->set_y(externalForce.Y());
    wrenchMsg.mutable_force()->set_z(externalForce.Z());

    // 메시지 게시
    this->forcePub.Publish(wrenchMsg);
  }
}

#include <gz/plugin/Register.hh>

IGNITION_ADD_PLUGIN(systems::ExternalForcePlugin,
                    System,
                    systems::ExternalForcePlugin::ISystemConfigure,
                    systems::ExternalForcePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(systems::ExternalForcePlugin,
                          "external_force_plugin::ExternalForcePlugin")

