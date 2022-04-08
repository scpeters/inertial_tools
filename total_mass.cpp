#include <iostream>

#include "sdf/Root.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"

#include "ignition/math/Inertial.hh"

int main(int argc, char *argv[]) {

  sdf::Root root;
  auto errors = root.Load(argv[1]);
  for (const auto &error : errors)
  {
    std::cerr << error << std::endl;
  }

  const sdf::Model * model = root.ModelByIndex(0);
  if (!model){ 
    std::cerr << "Unable to load model" << std::endl;
    return -1;
  }

  ignition::math::Inertiald totalInertial;

  for (int i = 0; i < model->LinkCount(); i++) {
    ignition::math::Pose3d linkPoseRelativeToModel;
    auto errors = model->LinkByIndex(i)->SemanticPose().Resolve(linkPoseRelativeToModel, "__model__");

    auto currentLinkInertial = model->LinkByIndex(i)->Inertial();
    currentLinkInertial.SetPose(linkPoseRelativeToModel * currentLinkInertial.Pose());
    totalInertial += currentLinkInertial;
  }

  auto totalMass = totalInertial.MassMatrix().Mass();
  auto xCentreOfMass = totalInertial.Pose().Pos().X();
  auto yCentreOfMass = totalInertial.Pose().Pos().Y();
  auto zCentreOfMass = totalInertial.Pose().Pos().Z();

  std::cout << "Total mass: " << totalMass << std::endl;

  std::cout << "Centre of mass: " << std::endl;
  std::cout << "X: " << xCentreOfMass << std::endl;
  std::cout << "Y: " << yCentreOfMass << std::endl;
  std::cout << "Z: " << zCentreOfMass << std::endl;

  std::cout << "Moment of Inertia: " << std::endl;
  std::cout << totalInertial.Moi() << std::endl;

  return 0;
}
