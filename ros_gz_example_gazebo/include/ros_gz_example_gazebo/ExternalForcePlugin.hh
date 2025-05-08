#ifndef EXTERNAL_FORCE_PLUGIN_HH_
#define EXTERNAL_FORCE_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include <memory>

namespace gz
{
namespace sim
{
namespace systems
{

class ExternalForcePlugin : public System,
                            public ISystemConfigure,
                            public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  // 모델 엔티티
  Entity modelEntity;

  // Joint 엔티티
  Entity jointEntity;

  // Baseline Force
  math::Vector3d baselineForce;

  // 외력 계산을 위한 Threshold
  double forceThreshold{0.5};

  // Gazebo 통신 노드
  transport::Node node;

  // Force 데이터 게시자
  transport::Node::Publisher forcePub;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif

