#include <gz/msgs/entity_wrench.pb.h>

#include <mutex>
#include <string>
#include <queue>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Util.hh"

#include "ros_gz_example_gazebo/ForceTorquePlugin.hh"

using namespace gz;
using namespace gz::sim;
using namespace systems;

class ignition::gazebo::systems::ForceTorquePluginPrivate
{
  /// \brief Callback for wrench subscription
  public: void OnWrench(const msgs::EntityWrench &_msg);

  /// \brief Callback for persistent wrench subscription
  public: void OnWrenchPersistent(const msgs::EntityWrench &_msg);

  /// \brief Callback for clearing persistent wrenches
  public: void OnWrenchClear(const msgs::Entity &_msg);

  /// \brief True if a console message should be printed whenever an
  /// instantaneous wrench is applied, a persistent wrench is cleared, etc.
  public: bool verbose{true};

  /// \brief Queue of incoming instantaneous wrenches
  public: std::queue<msgs::EntityWrench> newWrenches;

  /// \brief All persistent wrenches
  public: std::vector<msgs::EntityWrench> persistentWrenches;

  /// \brief Entities whose wrenches should be cleared
  public: std::queue<msgs::Entity> clearWrenches;

  /// \brief Communication node.
  public: transport::Node node;

  /// \brief A mutex to protect wrenches
  public: std::mutex mutex;
  
  /// ADDD
  /// \brief 마지막으로 받은 ROS2 Wrench 메시지
  public: std::optional<msgs::EntityWrench> lastWrench;

  /// \brief 새로운 ROS2 메시지를 수신했는지 여부
  public: bool newWrenchReceived{false};
  
};

/// \brief Extract wrench information from a message.
/// \param[in] _ecm Entity component manager
/// \param[in] _msg Entity message. If it's a link, that link is returned. If
/// it's a model, its canonical link is returned.
/// \param[out] Force to apply.
/// \param[out] Torque to apply.
/// \return Target link entity.
Link decomposeMessage(const EntityComponentManager &_ecm,
    const msgs::EntityWrench &_msg, math::Vector3d &_force,
    math::Vector3d &_torque)
{
  if (_msg.wrench().has_force_offset())
  {
    ignwarn << "Force offset currently not supported, it will be ignored."
            << std::endl;
  }

  if (_msg.wrench().has_force())
  {
    _force = msgs::Convert(_msg.wrench().force());
  }

  if (_msg.wrench().has_torque())
  {
    _torque = msgs::Convert(_msg.wrench().torque());
  }

  auto entity = entityFromMsg(_ecm, _msg.entity());
  if (entity == kNullEntity)
  {
    return Link();
  }

  Link link(entity);
  if (link.Valid(_ecm))
  {
    return link;
  }

  Model model(entity);
  if (model.Valid(_ecm))
  {
    return Link(model.CanonicalLink(_ecm));
  }

  ignerr << "Wrench can only be applied to a link or a model. Entity ["
         << entity << "] isn't either of them." << std::endl;
  return Link();
}

//////////////////////////////////////////////////
ForceTorquePlugin::ForceTorquePlugin()
  : dataPtr(std::make_unique<ForceTorquePluginPrivate>())
{
}

//////////////////////////////////////////////////
void ForceTorquePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto world = World(_entity);
  if (!world.Valid(_ecm))
  {
    ignerr << "ForceTorquePlugin system should be attached to a world."
           << std::endl;
    return;
  }

  this->dataPtr->verbose = _sdf->Get<bool>("verbose", true).first;

  // Initial wrenches
  for (auto elem = _sdf->FindElement("persistent");
       elem != nullptr;
       elem = elem->GetNextElement("persistent"))
  {
    msgs::EntityWrench msg;
    if (!elem->HasElement("entity_name") || !elem->HasElement("entity_type"))
    {
      ignerr << "Skipping <persistent> element missing entity name or type."
             << std::endl;
      continue;
    }

    msg.mutable_entity()->set_name(elem->Get<std::string>("entity_name"));

    auto typeStr = elem->FindElement("entity_type")->Get<std::string>();
    if (typeStr == "link")
    {
      msg.mutable_entity()->set_type(msgs::Entity::LINK);
    }
    else if (typeStr == "model")
    {
      msg.mutable_entity()->set_type(msgs::Entity::MODEL);
    }
    else
    {
      ignerr << "Skipping <persistent> element, entity type [" << typeStr
             << "] not supported." << std::endl;
      continue;
    }

    if (elem->HasElement("force"))
    {
      msgs::Set(msg.mutable_wrench()->mutable_force(),
          elem->FindElement("force")->Get<math::Vector3d>());
    }
    if (elem->HasElement("torque"))
    {
      msgs::Set(msg.mutable_wrench()->mutable_torque(),
          elem->FindElement("torque")->Get<math::Vector3d>());
    }
    this->dataPtr->OnWrenchPersistent(msg);
  }

  // Topic to apply wrench for one time step
  std::string topic{"/world/" + world.Name(_ecm).value() + "/wrench"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->dataPtr->node.Subscribe(topic, &ForceTorquePluginPrivate::OnWrench,
      this->dataPtr.get());

  ignmsg << "Listening to instantaneous wrench commands in [" << topic << "]"
         << std::endl;

  // Topic to apply wrench continuously
  topic = "/world/" + world.Name(_ecm).value() + "/wrench/persistent";
  if (_sdf->HasElement("topic_persistent"))
    topic = _sdf->Get<std::string>("topic_persistent");

  this->dataPtr->node.Subscribe(topic,
      &ForceTorquePluginPrivate::OnWrenchPersistent, this->dataPtr.get());

  ignmsg << "Listening to persistent wrench commands in [" << topic << "]"
         << std::endl;

  // Topic to clear persistent wrenches
  topic = "/world/" + world.Name(_ecm).value() + "/wrench/clear";
  if (_sdf->HasElement("topic_clear"))
    topic = _sdf->Get<std::string>("topic_clear");

  this->dataPtr->node.Subscribe(topic,
      &ForceTorquePluginPrivate::OnWrenchClear, this->dataPtr.get());

  ignmsg << "Listening to wrench clear commands in [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void ForceTorquePlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ForceTorquePlugin::PreUpdate");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // 시뮬레이션이 일시정지된 경우 종료
  if (_info.paused)
    return;

  // ROS2 메시지가 없는 경우에도 마지막 힘을 유지
  math::Vector3d force{0, 0, 0};
  math::Vector3d torque{0, 0, 0};

  if (this->dataPtr->newWrenchReceived)
  {
    // 새로운 메시지가 있으면 업데이트
    const auto &msg = this->dataPtr->lastWrench.value();
    force = msgs::Convert(msg.wrench().force());
    torque = msgs::Convert(msg.wrench().torque());

    // 새로운 메시지 처리 후 플래그 리셋
    this->dataPtr->newWrenchReceived = false;
  }
  else if (this->dataPtr->lastWrench.has_value())
  {
    // 새로운 메시지가 없으면 마지막 메시지를 사용
    const auto &msg = this->dataPtr->lastWrench.value();
    force = msgs::Convert(msg.wrench().force());
    torque = msgs::Convert(msg.wrench().torque());
  }
  else
  {
    // 메시지가 없으면 기본값(0 힘) 유지
    force = math::Vector3d(0, 0, 0);
    torque = math::Vector3d(0, 0, 0);
  }

  // 메시지에서 링크 추출 및 힘 적용
  if (this->dataPtr->lastWrench.has_value())
  {
    const auto &msg = this->dataPtr->lastWrench.value();
    auto link = decomposeMessage(_ecm, msg, force, torque);
    if (link.Valid(_ecm))
    {
      link.AddWorldWrench(_ecm, force, torque);

      if (this->dataPtr->verbose)
      {
        igndbg << "Applying wrench [" << force << " " << torque
               << "] to entity [" << link.Entity() << "]." << std::endl;
      }
    }
  }
}



//////////////////////////////////////////////////
void ForceTorquePluginPrivate::OnWrench(const msgs::EntityWrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!_msg.has_entity() || !_msg.has_wrench())
  {
    ignerr << "Missing entity or wrench in message: " << std::endl
           << _msg.DebugString() << std::endl;
    return;
  }

  this->newWrenches.push(_msg);

  // 마지막 데이터 업데이트
  this->lastWrench = _msg;
  this->newWrenchReceived = true;
}


//////////////////////////////////////////////////
void ForceTorquePluginPrivate::OnWrenchPersistent(const msgs::EntityWrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!_msg.has_entity() || !_msg.has_wrench())
  {
    ignerr << "Missing entity or wrench in message: " << std::endl
           << _msg.DebugString() << std::endl;
    return;
  }

  if (this->verbose)
  {
    igndbg << "Queueing persistent wrench:" << std::endl
           << _msg.DebugString() << std::endl;
  }

  this->persistentWrenches.push_back(_msg);
}

//////////////////////////////////////////////////
void ForceTorquePluginPrivate::OnWrenchClear(const msgs::Entity &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->clearWrenches.push(_msg);
}

IGNITION_ADD_PLUGIN(ForceTorquePlugin,
                    System,
                    ForceTorquePlugin::ISystemConfigure,
                    ForceTorquePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ForceTorquePlugin,
                          "force_torque_plugin::ForceTorquePlugin")


