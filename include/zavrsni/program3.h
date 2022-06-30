#ifndef ZAVRSNI_PROGRAM3_H
#define ZAVRSNI_PROGRAM3_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <zavrsni/example.h>

namespace zavrsni
{
/**
 * @brief An example of a robot leveraging trajopt and tesseract to
 * create an optimal motion trajectory for a complex cartesian path.
 */
class Program3 : public Example
{
public:
  Program3(tesseract_environment::Environment::Ptr env,
                     tesseract_visualization::Visualization::Ptr plotter = nullptr);
  ~Program3() override = default;
  Program3(const Program3&) = default;
  Program3& operator=(const Program3&) = default;
  Program3(Program3&&) = default;
  Program3& operator=(Program3&&) = default;

  tesseract_common::JointTrajectory run() override final;

private:
  static tesseract_common::VectorIsometry3d makePuzzleToolPoses(const tesseract_common::ResourceLocator::ConstPtr& locator);
};

}  // namespace zavrsni
#endif  // ZAVRSNI_PROGRAM3_H
