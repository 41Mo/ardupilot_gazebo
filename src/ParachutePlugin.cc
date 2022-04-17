#include "ParachutePlugin.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/physics.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;
void ParachutePlugin::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_em)
{
  this->model = Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    ignerr << "ParachutePlugin should be attached to a model "
      << "entity. Failed to initialize." << "\n";
    return;
  }

  igndbg << this->model.Name(_ecm) << std::endl;
  if (_sdf->HasElement("parent_link"))
  {
    auto parentLinkName = _sdf->Get<std::string>("parent_link");
    this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
    if (kNullEntity == this->parentLinkEntity)
    {
      ignerr << "Link with name " << parentLinkName
             << " not found in model " << this->model.Name(_ecm)
             << ". Make sure the parameter 'parent_link' has the "
             << "correct value. Failed to initialize.\n";
      return;
    }
  }
  else
  {
    ignerr << "'parent_link' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    ignerr << "'child_model' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    ignerr << "'child_link' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  std::vector<std::string> topics;
  // detach topic configure
  if (_sdf->HasElement("detach_topic"))
  {
    topics.push_back(_sdf->Get<std::string>("detach_topic"));
  }
  topics.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/detach");
  this->detach_topic = validTopic(topics);

  std::vector<std::string> topics1;

  // attach topic configure
  if (_sdf->HasElement("attach_topic"))
  {
    topics1.push_back(_sdf->Get<std::string>("attach_topic"));
  }
  topics1.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/attach");
  this->attach_topic = validTopic(topics1);

  this->suppressChildWarning =
    _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning).first;


  this->node.Subscribe(
      this->detach_topic, &ParachutePlugin::OnDetachRequest, this);

  ignmsg << "ParachutePlugin subscribing to messages on "
         << "[" << this->detach_topic << "]" << std::endl;

  this->node.Subscribe(
      this->attach_topic, &ParachutePlugin::OnAttachRequest, this);

  ignmsg << "ParachutePlugin subscribing to messages on "
         << "[" << this->attach_topic << "]" << std::endl;

  this->node.Subscribe("/parachute/start", &ParachutePlugin::OnStartRequest, this);
  this->validConfig = true;
}

void ParachutePlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  //IGN_PROFILE("ParachutePlugin::PreUpdate");

  if (this->validConfig && !this->attached && this->should_attach)
  {

    Entity parachute_entity{kNullEntity};
    if ("__model__" == this->childModelName)
    {
      parachute_entity = this->model.Entity();
    }
    else
    {
      parachute_entity = _ecm.EntityByComponents(
          components::Model(), components::Name(this->childModelName));
    }

    if (kNullEntity != parachute_entity)
    {
      this->childLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parachute_entity),
          components::Name(this->childLinkName));

      math::Quaterniond rpy{0, 2.35, 0};
      math::Vector3d xyz{0, -0.12, 0};
      auto rot = worldPose(this->model.Entity(), _ecm).CoordRotationSub(rpy);
      auto pos = worldPose(this->model.Entity(), _ecm).CoordPositionAdd(xyz);
      math::Pose3d pose{
        pos,
        rot,
      };
      auto parachute_model = Model(parachute_entity);
      parachute_model.SetWorldPoseCmd(
        _ecm,
        pose
      );

      auto chute_pos = worldPose(parachute_entity, _ecm);

      if (!init_pos_saved) {
        initial_pos = chute_pos;
        init_pos_saved = true;
      }
      
      if (initial_pos != chute_pos) {
        igndbg << "Model OK\n";
        model_ok = true;
      }

      if (kNullEntity != this->childLinkEntity && model_ok)
      {

        // Attach the models
        // We do this by creating a detachable joint entity.
        this->detachableJointEntity = _ecm.CreateEntity();

        _ecm.CreateComponent(
            this->detachableJointEntity,
            components::DetachableJoint({this->parentLinkEntity,
                                         this->childLinkEntity, "fixed"}));
        this->attached = true;
        this->should_attach = false;
      }
      else if (kNullEntity == this->childLinkEntity)
      {
        ignwarn << "Child Link " << this->childLinkName
                << " could not be found.\n";
      }
    }
  }

  if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
  {
    // Detach the models
    igndbg << "Removing entity: " << this->detachableJointEntity << std::endl;
    _ecm.RequestRemoveEntity(this->detachableJointEntity);
    this->detachableJointEntity = kNullEntity;
    this->detachRequested = false;
    attached = false;
  } 
  if (this->attachRequested && (kNullEntity == this->detachableJointEntity))
  {
    // Detach the models
    igndbg << "Attach requested! " << std::endl;
    this->should_attach = true;
    this->attachRequested = false;
  }
}

//////////////////////////////////////////////////
void ParachutePlugin::OnDetachRequest(const msgs::Empty &)
{
  this->detachRequested = true;
}

void ParachutePlugin::OnAttachRequest(const msgs::Empty &)
{
  this->attachRequested = true;
}

void ParachutePlugin::OnStartRequest(const msgs::Empty &)
{
  this->start = true;
}

IGNITION_ADD_PLUGIN(ParachutePlugin,
                    ignition::gazebo::System,
                    ParachutePlugin::ISystemConfigure,
                    ParachutePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParachutePlugin,"ParachutePlugin")