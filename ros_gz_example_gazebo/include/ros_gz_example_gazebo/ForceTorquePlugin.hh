#ifndef GZ_SIM_SYSTEMS_FORCETORQUEPLUGIN_HH_
#define GZ_SIM_SYSTEMS_FORCETORQUEPLUGIN_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
  // Forward declaration
  class ForceTorquePluginPrivate;

  /// \brief Exposes transport topics and SDF params for applying forces and
  /// torques to links in simulation. It should be attached to a world.
  ///
  /// The target link is defined in each message. If a link entity is provided,
  /// that will receive a wrench. If a model is provided, its canonical link
  /// will receive it. No other entity types are supported.
  ///
  /// ## Topics
  ///
  /// * /world/<world_name>/wrench
  ///     * Message type: msgs::EntityWrench
  ///     * Effect: Applies the given wrench during a single time step.
  ///
  /// * /world/<world_name>/wrench/persistent
  ///     * Message type: msgs::EntityWrench
  ///     * Effect: Keeps applying the given wrench every time step. Persistent
  ///               wrenches can be applied to entities that aren't in
  ///               simulation yet, and will start taking effect once they do.
  ///
  /// * /world/<world_name>/wrench/clear
  ///     * Message type: msgs::Entity
  ///     * Effect: Clears any persistent wrenches that are being applied to
  ///               the given entity.
  ///
  /// ## System Parameters
  ///
  /// Persistent wrenches can be defined from SDF, for example:
  ///
  /// ```
  /// <persistent>
  ///   <entity_name>box</entity_name>
  ///   <entity_type>model</entity_type>
  ///   <force>-10 0 0</force>
  ///   <torque>0 0 0.1</torque>
  /// </persistent>
  /// ```
  class ForceTorquePlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ForceTorquePlugin();

    /// \brief Destructor
    public: ~ForceTorquePlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ForceTorquePluginPrivate> dataPtr;
  };
}
}
}
}

#endif

